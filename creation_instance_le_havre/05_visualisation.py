"""Carte interactive des instances (HTML + Leaflet), servie en local et ouverte dans le navigateur.

Affiche, pour l'instance choisie dans le menu : les pickups et deliveries reliés par une ligne pour chaque
requête, le dépôt, les stations et le tracé des lignes de tram A, B et C (OpenStreetMap). Un clic sur une
requête (carte ou tableau) donne son détail. Avec une solution de resolution_tc (jour + critère) ou de idarp_algo
(results/lehavre_rural/<modèle>/<jour>/sol_<instance>.txt, écrite par main_LNS -sol, ou solution initiale de
main_nb_vehicles dans results/lehavre_sol_init/<modèle>/<jour>/), chaque requête est tracée
selon son trajet : véhicule, bus ou tram, marche, et un symbole à chaque montée ou descente.

  .venv/bin/python 05_visualisation.py                 # toutes les instances de lehavre_rural
  .venv/bin/python 05_visualisation.py -i 30_30_3      # instance sélectionnée à l'ouverture

La page est servie sur http://localhost (Ctrl+C pour arrêter) : ouverte directement depuis le disque
(file://), les serveurs de tuiles refusent d'afficher le fond de carte.
"""
import argparse
import functools
import json
import webbrowser
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

import geopandas as gpd
import osmium
import osmnx as ox
import pandas as pd

import gtfs
import solution_idarp
from config import DEPOT, INSTANCES_DIR, PREP_DIR, RAW_DIR, ROOT

TC_RESULTS_DIR = ROOT.parent / "results" / "lehavre_rural_tc"   # solutions de resolution_tc
# Solutions de idarp_algo, par modèle (widarp, idarp, darp) et par jour :
IDARP_RESULTS_DIR = ROOT.parent / "results" / "lehavre_rural"   # main_LNS -sol : <modèle>/<jour>/sol_<instance>.txt
IDARP_INIT_DIR = ROOT.parent / "results" / "lehavre_sol_init"   # main_nb_vehicles -sol : idem, sol_init_<instance>.txt
MODEL_LABELS = {"widarp": "wIDARP", "idarp": "IDARP", "darp": "DARP"}
TC_DAYS = {"scolaire": "lundi scolaire", "vacances": "lundi hors scolaire", "samedi": "samedi"}
TRAM_RELATIONS = {"A": 2641214, "B": 2641215, "C": 20185321}   # relations OSM "Tram X : terminus -> terminus"
TRAM_COLORS = {"A": "#d1495b", "B": "#2e86ab", "C": "#5b8c2a"}


def tram_lines():
    """Tracé des lignes de tram (liste de polylignes [lat, lon] par ligne), mis en cache."""
    cache = PREP_DIR / "tram_lignes.json"
    if cache.exists():
        lines = json.loads(cache.read_text())
        if set(lines) == set(TRAM_RELATIONS):
            return lines
    pbf = str(RAW_DIR / "haute-normandie.osm.pbf")
    members = {line: [] for line in TRAM_RELATIONS}
    wanted = {rel: line for line, rel in TRAM_RELATIONS.items()}
    for rel in osmium.FileProcessor(pbf, osmium.osm.RELATION):
        if rel.id in wanted:
            members[wanted[rel.id]] = [m.ref for m in rel.members if m.type == "w"]
    way_ids = {w for ws in members.values() for w in ws}
    coords = {}
    ways = osmium.FileProcessor(pbf, osmium.osm.NODE | osmium.osm.WAY).with_locations() \
        .with_filter(osmium.filter.EntityFilter(osmium.osm.WAY)).with_filter(osmium.filter.IdFilter(way_ids))
    for way in ways:
        if way.id in way_ids:
            coords[way.id] = [[n.lat, n.lon] for n in way.nodes if n.location.valid()]
    lines = {line: [coords[w] for w in ws if w in coords] for line, ws in members.items()}
    cache.write_text(json.dumps(lines))
    return lines


def stations_with_lines():
    stations = gpd.read_file(PREP_DIR / "tram_stations.geojson")
    routes = gtfs.read("routes.txt", dtype=str)
    trips = gtfs.read("trips.txt", dtype=str)
    st = gtfs.read("stop_times.txt", dtype=str, usecols=["trip_id", "stop_id"])
    st = st.merge(trips[["trip_id", "route_id"]]).merge(routes[["route_id", "route_short_name", "route_type"]])
    st = st[st.route_type == "0"]
    lines_by_stop = st.groupby("stop_id").route_short_name.agg(lambda s: sorted(set(s)))
    out = []
    for s in stations.itertuples():
        lines = sorted({l for sid in s.stop_ids.split(";") for l in lines_by_stop.get(sid, [])})
        out.append({"nom": s.name, "lat": s.lat, "lon": s.lon, "lignes": lines})
    return out


def instances_data():
    communes = gpd.read_file(RAW_DIR / "communes_cu.geojson").set_index("code").nom.to_dict()
    data = {}
    for r_file in sorted(INSTANCES_DIR.glob("r*.csv"), key=lambda p: [int(x) for x in p.stem[1:].split("_")]):
        name = r_file.stem[1:]
        reqs = pd.read_csv(r_file, dtype={"commune_rurale": str})
        data[name] = [{
            "id": int(q.requete), "sens": q.sens, "type_fenetre": q.type_fenetre,
            "heure": q.heure_reference, "fenetre_pickup": q.fenetre_pickup, "fenetre_delivery": q.fenetre_delivery,
            "quantite": int(q.quantite), "voiture": int(q.temps_direct_voiture), "mrt": int(q.mrt),
            "commune": communes.get(q.commune_rurale, q.commune_rurale),
            "marche_rural": q.marche_tram_rural, "marche_ville": q.marche_tram_ville,
            "p": [q.pickup_lat, q.pickup_lon], "d": [q.delivery_lat, q.delivery_lon],
        } for q in reqs.itertuples()]
    return data


def solutions(instances):
    """Solutions disponibles, par source : {clé: {"label", "kind": "tc" | "idarp", "instances": {...}}}.
    TC (resolution_tc) : instance -> requête -> critère -> trajet ; wIDARP (main_LNS -sol) : instance ->
    {"requetes": requête -> trajet, "routes": tournées}."""
    out = {}
    for day, label in TC_DAYS.items():
        for name in instances:
            f = TC_RESULTS_DIR / day / f"solutions_{name}.json"
            if f.exists():
                out.setdefault(f"tc:{day}", {"label": f"TC – {label}", "kind": "tc", "instances": {}})[
                    "instances"][name] = json.loads(f.read_text())["requetes"]
    files = [(f, "", "") for f in sorted(IDARP_RESULTS_DIR.glob("*/*/sol_*.txt"))] + \
            [(f, "init-", " initiale") for f in sorted(IDARP_INIT_DIR.glob("*/*/sol_init_*.txt"))]
    walk_graph = ox.load_graphml(PREP_DIR / "walk.graphml") if files else None
    for f, prefix, suffix in files:
        model = f.parent.parent.name
        label = MODEL_LABELS.get(model, model) + suffix
        sol = solution_idarp.read(f)
        name = sol["instance"][1:-4]
        if name not in instances:
            continue
        mode = "horaires" if sol["pt"][0] == "tram" else "matrice"
        key = f"{prefix}{model}:{f.parent.name}"
        out.setdefault(key, {"label": f"{label} – {f.parent.name.replace('_', ' ')} ({mode})", "kind": "idarp",
                             "instances": {}})["instances"][name] = solution_idarp.trips_for_map(sol, walk_graph)
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("-i", "--instance", help="instance affichée à l'ouverture (ex. 30_30_0)")
    parser.add_argument("-p", "--port", type=int, default=8765)
    parser.add_argument("--sans-ouvrir", action="store_true", help="écrire la page sans lancer le serveur")
    args = parser.parse_args()

    instances = instances_data()
    payload = {
        "instances": instances,
        "solutions": solutions(instances),
        "stations": stations_with_lines(),
        "tram": tram_lines(),
        "tramColors": TRAM_COLORS,
        "depot": [DEPOT[1], DEPOT[0]],
        "initial": args.instance,
    }
    if not payload["instances"]:
        raise SystemExit(f"Aucune instance (r*.csv) dans {INSTANCES_DIR}")
    template = (ROOT / "visualisation_template.html").read_text()
    out = INSTANCES_DIR / "visualisation.html"
    out.write_text(template.replace("/*__DATA__*/null", json.dumps(payload)))
    print(f"Carte écrite dans {out}")
    if args.sans_ouvrir:
        return

    class QuietHandler(SimpleHTTPRequestHandler):
        def log_message(self, *_):
            pass

    handler = functools.partial(QuietHandler, directory=str(INSTANCES_DIR))
    with ThreadingHTTPServer(("localhost", args.port), handler) as server:
        url = f"http://localhost:{args.port}/{out.name}" + (f"#{args.instance}" if args.instance else "")
        print(f"Carte sur {url} (Ctrl+C pour arrêter)")
        webbrowser.open(url)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
