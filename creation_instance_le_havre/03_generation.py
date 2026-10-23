"""Génère des instances aléatoires au format wIDARP dans instances/lehavre_rural/.

Les mêmes requêtes servent pour les trois jours de référence ; seul le temps en tram en dépend.
Chaque instance se déroule entièrement le matin (7h-11h) ou le soir (15h-19h), période tirée au hasard
(récapitulatif dans instances.csv). Pour chaque instance k (n requêtes) :
  i{n}_{n}_{k}.txt   requêtes (format lu par idarp_algo)
  d{n}_{n}_{k}.txt   temps en voiture (min) entre dépôt, pickups/deliveries et stations de tram
  w{n}_{n}_{k}.txt   temps à pied (min), mêmes points
  l{n}_{n}_{k}.geojson  localisation des points (même indexation)
  r{n}_{n}_{k}.csv   description des requêtes (sens, fenêtre d'origine, communes...) pour le mode TC
et, pour chaque jour, public_transport_time_<jour>.txt (temps en tram entre stations, horaires du jour)
et tram_runs_<jour>.txt (chaque course de tram avec ses heures de passage, option -tram de idarp_algo).
Une requête dont le trajet direct en voiture fait moins de MIN_DIRECT_DRIVE min est rejetée et tirée à nouveau.

Indexation des points : 0 dépôt, 2r+1 pickup et 2r+2 delivery de la requête r, puis 1+2n+s station s.
"""
import argparse
from datetime import date

import geopandas as gpd
import numpy as np
import osmnx as ox
import pandas as pd
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

import gtfs
from config import (CAPACITY, DAYS, DEPOT, HORIZON, INSTANCES_DIR, MIN_DIRECT_DRIVE, MRT_RATIO,
                    PEAK_SHARE, PERIODS, PREP_DIR, QUANTITY_PROBS, SERVICE_TIME, TIME_ORIGIN, TW_WIDTH,
                    WALK_SPEED_KMH)
from temps_voiture import set_realistic_travel_times

WALK_M_PER_MIN = WALK_SPEED_KMH * 1000 / 60


class Network:
    """Graphe osmnx converti en matrice creuse pour des Dijkstra rapides."""

    def __init__(self, graph, weight, to_minutes, directed):
        self.graph = graph
        self.index = {n: i for i, n in enumerate(graph.nodes)}
        edges = pd.DataFrame([(self.index[u], self.index[v], float(d[weight]) * to_minutes)
                              for u, v, d in graph.edges(data=True)], columns=["u", "v", "t"])
        edges = edges.groupby(["u", "v"]).t.min().reset_index()  # arêtes parallèles : la plus rapide
        self.matrix = csr_matrix((edges.t, (edges.u, edges.v)), shape=(len(self.index),) * 2)
        self.directed = directed

    def nearest(self, lon, lat):
        return ox.distance.nearest_nodes(self.graph, lon, lat)

    def times(self, nodes):
        """Matrice des temps (min, entiers arrondis) entre les nœuds donnés, dans l'ordre."""
        idx = [self.index[n] for n in nodes]
        full = dijkstra(self.matrix, directed=self.directed, indices=idx)
        return np.rint(full[:, idx]).astype(int)

    def time(self, origin, destination):
        return self.times([origin, destination])[0, 1]


def tram_stop_times(day, stations):
    """Passages des trams du jour (GTFS) : course, station, heures d'arrivée et de départ (min depuis minuit)."""
    yyyymmdd = day.replace("-", "")
    cal = gtfs.read("calendar_dates.txt", dtype=str)
    services = set(cal[(cal.date == yyyymmdd) & (cal.exception_type == "1")].service_id)
    routes = gtfs.read("routes.txt")
    trips = gtfs.read("trips.txt", dtype={"service_id": str})
    trips = trips[trips.service_id.isin(services) & trips.route_id.isin(routes[routes.route_type == 0].route_id)]
    st = gtfs.read("stop_times.txt", dtype={"stop_id": str})
    st = st[st.trip_id.isin(trips.trip_id)].copy()
    to_min = lambda s: s.str.split(":").map(lambda x: int(x[0]) * 60 + int(x[1]))
    st["arr"], st["dep"] = to_min(st.arrival_time), to_min(st.departure_time)
    stop_to_station = {sid: row.station for row in stations.itertuples() for sid in row.stop_ids.split(";")}
    st["station"] = st.stop_id.map(stop_to_station)
    return st.merge(trips[["trip_id", "route_id"]], on="trip_id").merge(routes[["route_id", "route_short_name"]])


def tram_runs(day, stations):
    """Lignes du fichier -tram de idarp_algo : le nombre de courses, puis une course par ligne,
    "ligne nbArrêts station heure station heure ...", heures en minutes depuis TIME_ORIGIN."""
    lines = []
    for _, trip in tram_stop_times(day, stations).sort_values("stop_sequence").groupby("trip_id", sort=False):
        if trip.dep.iloc[0] < TIME_ORIGIN or trip.arr.iloc[-1] > TIME_ORIGIN + HORIZON:
            continue
        stops = " ".join(f"{s} {t - TIME_ORIGIN}" for s, t in zip(trip.station, trip.dep))
        lines.append((trip.dep.iloc[0], f"{trip.route_short_name.iloc[0]} {len(trip)} {stops}"))
    lines.sort()
    return [str(len(lines))] + [line for _, line in lines]


def tram_times(day, stations):
    """Temps en tram (min) entre stations : meilleur temps en véhicule parmi les courses du jour
    entre TIME_ORIGIN et TIME_ORIGIN + HORIZON, correspondances A/B comprises (Floyd-Warshall)."""
    st = tram_stop_times(day, stations)
    n = len(stations)
    pt = np.full((n, n), np.inf)
    np.fill_diagonal(pt, 0)
    for _, trip in st.sort_values("stop_sequence").groupby("trip_id"):
        if trip.dep.iloc[0] < TIME_ORIGIN or trip.arr.iloc[-1] > TIME_ORIGIN + HORIZON:
            continue
        s, dep, arr = trip.station.to_numpy(), trip.dep.to_numpy(), trip.arr.to_numpy()
        for i in range(len(s)):
            for j in range(i + 1, len(s)):
                pt[s[i], s[j]] = min(pt[s[i], s[j]], arr[j] - dep[i])
    for k in range(n):
        pt = np.minimum(pt, pt[:, k, None] + pt[None, k, :])
    if np.isinf(pt).any():
        raise ValueError(f"Stations non reliées en tram le {day}")
    return pt.astype(int)


def draw_time(rng, period):
    """Heure de référence (min depuis minuit) dans la période de l'instance, pic avec probabilité PEAK_SHARE."""
    start, end, peak_start, peak_end = period
    if rng.random() < PEAK_SHARE:
        return int(rng.integers(peak_start, peak_end + 1)), (start, end)
    # Hors pic : uniforme sur la période privée du pic.
    off_peak = (peak_start - start) + (end - peak_end)
    x = int(rng.integers(0, off_peak + 1))
    return (start + x if x <= peak_start - start else peak_end + x - (peak_start - start)), (start, end)


def draw_requests(rng, n, period, rural, carreaux, city, drive_net):
    car_p = carreaux.weight.to_numpy() / carreaux.weight.sum()
    rural_by_car = rural.groupby("idcar_200m")
    city_by_station = city.groupby("nearest_station")
    city_stations = list(city_by_station.groups)
    quantities, q_probs = list(QUANTITY_PROBS), list(QUANTITY_PROBS.values())

    reqs, nb_rejected = [], 0
    while len(reqs) < n:
        car = carreaux.idcar_200m.iloc[rng.choice(len(carreaux), p=car_p)]
        rural_pt = rural_by_car.get_group(car).sample(1, random_state=rng).iloc[0]
        station = city_stations[rng.integers(len(city_stations))]
        city_pt = city_by_station.get_group(station).sample(1, random_state=rng).iloc[0]
        inbound = rng.random() < 0.5
        origin, destination = (rural_pt, city_pt) if inbound else (city_pt, rural_pt)
        if drive_net.time(origin.drive_node, destination.drive_node) < MIN_DIRECT_DRIVE:
            nb_rejected += 1
            continue
        h, (start, end) = draw_time(rng, period)
        departure = rng.random() < 0.5
        # La fenêtre de 15 min doit tenir dans la période.
        h = min(max(h, start), end - TW_WIDTH) if departure else min(max(h, start + TW_WIDTH), end)
        reqs.append({
            "requete": len(reqs),
            "sens": "entrant" if inbound else "sortant",
            "pickup": origin,
            "delivery": destination,
            "type_fenetre": "depart" if departure else "arrivee",
            "heure_reference": h,
            "quantite": int(rng.choice(quantities, p=q_probs)),
            "commune_rurale": rural_pt.code,
            "marche_tram_rural": round(rural_pt.walk_to_tram, 1),
            "marche_tram_ville": round(city_pt.walk_to_tram, 1),
        })
    return reqs, nb_rejected


def hhmm(minutes_since_origin):
    m = minutes_since_origin + TIME_ORIGIN
    return f"{m // 60:02d}:{m % 60:02d}"


def build_instance(reqs, walk_net, drive_net, depot_nodes, station_nodes):
    walk_nodes = [depot_nodes[0]] + [p for q in reqs for p in (q["pickup"].osmid, q["delivery"].osmid)] \
        + station_nodes[0]
    drive_nodes = [depot_nodes[1]] + [p for q in reqs for p in (q["pickup"].drive_node, q["delivery"].drive_node)] \
        + station_nodes[1]
    walk_t, drive_t = walk_net.times(walk_nodes), drive_net.times(drive_nodes)

    lines = []
    for q in reqs:
        p, d = 2 * q["requete"] + 1, 2 * q["requete"] + 2
        direct = int(drive_t[p, d])
        mrt = int(MRT_RATIO * direct + SERVICE_TIME)
        h = q["heure_reference"] - TIME_ORIGIN
        if q["type_fenetre"] == "depart":
            tw_p = (h, h + TW_WIDTH)
            tw_d = (h + direct, h + TW_WIDTH + mrt)
        else:
            tw_d = (h - TW_WIDTH, h)
            tw_p = (h - TW_WIDTH - mrt, h - direct)
        q.update(temps_direct_voiture=direct, mrt=mrt, tw_pickup=tw_p, tw_delivery=tw_d)
        lines.append(f"{p} {tw_p[0]} {tw_p[1]} {d} {tw_d[0]} {tw_d[1]} {mrt} {q['quantite']} {SERVICE_TIME}")
    return lines, walk_t, drive_t


def write_matrix(path, m):
    path.write_text("\n".join(" ".join(map(str, row)) for row in m))


def write_instance(folder, name, n, reqs, lines, walk_t, drive_t, depot, stations):
    nb_stations = len(stations)
    (folder / f"i{name}.txt").write_text(f"{n} {n} {nb_stations} {CAPACITY} {HORIZON}\n" + "\n".join(lines) + "\n")
    write_matrix(folder / f"d{name}.txt", drive_t)
    write_matrix(folder / f"w{name}.txt", walk_t)

    rows = [{"index": 0, "type": "depot", "requete": -1, "lon": depot[0], "lat": depot[1]}]
    for q in reqs:
        for role in ("pickup", "delivery"):
            pt = q[role]
            rows.append({"index": 2 * q["requete"] + (1 if role == "pickup" else 2), "type": role,
                         "requete": q["requete"], "lon": pt.x, "lat": pt.y})
    for s in stations.itertuples():
        rows.append({"index": 1 + 2 * n + s.station, "type": "station", "requete": -1, "nom": s.name,
                     "lon": s.lon, "lat": s.lat})
    pts = pd.DataFrame(rows)
    gpd.GeoDataFrame(pts, geometry=gpd.points_from_xy(pts.lon, pts.lat), crs="EPSG:4326") \
        .to_file(folder / f"l{name}.geojson")

    desc = pd.DataFrame([{
        "requete": q["requete"], "sens": q["sens"], "type_fenetre": q["type_fenetre"],
        "heure_reference": hhmm(q["heure_reference"] - TIME_ORIGIN),
        "fenetre_pickup": f"{hhmm(q['tw_pickup'][0])}-{hhmm(q['tw_pickup'][1])}",
        "fenetre_delivery": f"{hhmm(q['tw_delivery'][0])}-{hhmm(q['tw_delivery'][1])}",
        "quantite": q["quantite"], "temps_direct_voiture": q["temps_direct_voiture"], "mrt": q["mrt"],
        "pickup_lon": q["pickup"].x, "pickup_lat": q["pickup"].y,
        "delivery_lon": q["delivery"].x, "delivery_lat": q["delivery"].y,
        "commune_rurale": q["commune_rurale"],
        "marche_tram_rural": q["marche_tram_rural"], "marche_tram_ville": q["marche_tram_ville"],
    } for q in reqs])
    desc.to_csv(folder / f"r{name}.csv", index=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("-n", "--nb-requetes", type=int, default=30)
    parser.add_argument("-k", "--nb-instances", type=int, default=10)
    parser.add_argument("-j", "--jours", nargs="+", default=list(DAYS), choices=list(DAYS))
    parser.add_argument("-s", "--graine", type=int, default=0)
    args = parser.parse_args()

    print("Chargement des données préparées...")
    rural = pd.read_csv(PREP_DIR / "rural_nodes.csv", dtype={"code": str})
    city = pd.read_csv(PREP_DIR / "city_nodes.csv", dtype={"code": str})
    carreaux = gpd.read_file(PREP_DIR / "carreaux_ruraux.geojson")
    stations = gpd.read_file(PREP_DIR / "tram_stations.geojson")
    walk_g, drive_g = ox.load_graphml(PREP_DIR / "walk.graphml"), ox.load_graphml(PREP_DIR / "drive.graphml")
    set_realistic_travel_times(drive_g)
    walk_net = Network(walk_g, "length", 1 / WALK_M_PER_MIN, directed=False)
    drive_net = Network(drive_g, "travel_time", 1 / 60, directed=True)
    depot_nodes = (walk_net.nearest(*DEPOT), drive_net.nearest(*DEPOT))
    station_nodes = (list(walk_net.nearest(stations.lon, stations.lat)),
                     list(drive_net.nearest(stations.lon, stations.lat)))

    folder = INSTANCES_DIR
    folder.mkdir(parents=True, exist_ok=True)
    stations.to_file(folder / "tram_stations.geojson")
    jours = []
    for day_name in args.jours:
        day = DAYS[day_name]
        write_matrix(folder / f"public_transport_time_{day_name}.txt", tram_times(day, stations))
        (folder / f"tram_runs_{day_name}.txt").write_text("\n".join(tram_runs(day, stations)) + "\n")
        jours.append(f"{day_name} {day} ({date.fromisoformat(day).strftime('%A')})")
    (folder / "jours.txt").write_text("\n".join(jours) + "\n")

    # Une instance se déroule entièrement le matin ou le soir (période tirée au hasard).
    n = args.nb_requetes
    summary = []
    for k in range(args.nb_instances):
        rng = np.random.default_rng([args.graine, n, k])
        period = PERIODS[rng.integers(len(PERIODS))]
        period_name = "matin" if period[0] < 12 * 60 else "soir"
        reqs, nb_rejected = draw_requests(rng, n, period, rural, carreaux, city, drive_net)
        lines, walk_t, drive_t = build_instance(reqs, walk_net, drive_net, depot_nodes, station_nodes)
        write_instance(folder, f"{n}_{n}_{k}", n, reqs, lines, walk_t, drive_t, DEPOT, stations)
        summary.append({"instance": f"{n}_{n}_{k}", "periode": period_name,
                        "debut": f"{period[0] // 60}h", "fin": f"{period[1] // 60}h"})
        print(f"instance {k} ({period_name}) : {nb_rejected} requêtes tirées puis rejetées "
              f"(< {MIN_DIRECT_DRIVE} min en voiture)")
    summary_file = folder / "instances.csv"
    old = pd.read_csv(summary_file) if summary_file.exists() else pd.DataFrame(columns=["instance"])
    pd.concat([old[~old.instance.isin([x["instance"] for x in summary])], pd.DataFrame(summary)]) \
        .to_csv(summary_file, index=False)
    print(f"{args.nb_instances} instances de {n} requêtes -> {folder}")


if __name__ == "__main__":
    main()
