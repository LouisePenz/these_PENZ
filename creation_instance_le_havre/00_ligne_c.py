"""Crée GTFS_ligne_C/ : la future ligne de tram C, au format GTFS, lue avec le GTFS LiA.

- Arrêts : stop_locations.geojson (index 23 à 39 ; Université et Gares sont partagés avec A et B et
  portent les mêmes noms que dans le GTFS, ils sont donc regroupés avec leurs stations).
- Temps de parcours : temps cumulés depuis le terminus Parc Jardin, lus dans l'ancienne matrice
  instances/lehavre/public_transport_time.txt (m[Parc Jardin][arrêt], fidèles au GTFS pour les lignes
  A et B, contrairement aux temps entre arrêts consécutifs, arrondis par défaut).
- Horaires : une course C pour chaque course A du GTFS, même service (jour) et même heure de départ au
  terminus ; Parc Jardin joue le rôle de Grand Hameau, Vallée Béreult celui de La Plage.
"""
import geopandas as gpd
import pandas as pd

from config import GTFS_DIR, GTFS_LIGNE_C_DIR, ROOT

# De Parc Jardin à Vallée Béreult : (index dans stop_locations.geojson, minutes depuis Parc Jardin)
LINE_C = [(23, 0), (24, 1), (25, 3), (26, 5), (27, 8), (28, 10), (29, 13), (30, 14), (31, 16), (32, 18),
          (33, 19), (34, 21), (7, 23), (12, 24), (35, 26), (36, 28), (37, 29), (38, 30), (39, 32)]
ROUTE_A = "8-A"
ROUTE_C = "8-C"


def hms(minutes):
    minutes = int(round(minutes))
    return f"{minutes // 60:02d}:{minutes % 60:02d}:00"


def main():
    locations = gpd.read_file(ROOT / "stop_locations.geojson").set_index("index")
    stops = pd.DataFrame([{
        "stop_id": f"C{i}", "stop_code": "", "stop_name": locations.name[i],
        "stop_lat": round(locations.geometry[i].y, 6), "stop_lon": round(locations.geometry[i].x, 6),
        "location_type": "", "parent_station": "", "wheelchair_boarding": 1,
    } for i, _ in LINE_C])
    # Noms identiques au GTFS LiA pour les stations partagées (regroupement par nom).
    gtfs_names = set(pd.read_csv(GTFS_DIR / "stops.txt", dtype=str).stop_name)
    for name in ("Université", "Gares"):
        assert name in gtfs_names and name in set(stops.stop_name), name

    routes = pd.DataFrame([{"route_id": ROUTE_C, "agency_id": "LIA", "route_short_name": "C",
                            "route_long_name": "Parc Jardin - Vallee Bereult", "route_type": 0,
                            "route_color": "", "route_text_color": "", "route_url": ""}])

    trips_a = pd.read_csv(GTFS_DIR / "trips.txt", dtype=str)
    trips_a = trips_a[trips_a.route_id == ROUTE_A]
    st = pd.read_csv(GTFS_DIR / "stop_times.txt", dtype=str)
    first = st[st.trip_id.isin(trips_a.trip_id) & (st.stop_sequence.astype(int) == 1)].set_index("trip_id")
    to_min = lambda s: int(s[:2]) * 60 + int(s[3:5]) + int(s[6:8]) / 60

    trips, stop_times = [], []
    for a in trips_a.itertuples():
        start = to_min(first.departure_time[a.trip_id])
        # A direction 0 part de Grand Hameau : C part de Parc Jardin ; direction 1 : de Vallée Béreult.
        sequence = LINE_C if a.direction_id == "0" else [(i, LINE_C[-1][1] - t) for i, t in reversed(LINE_C)]
        trip_id = f"C-{a.trip_id}"
        trips.append({"route_id": ROUTE_C, "service_id": a.service_id, "trip_id": trip_id,
                      "trip_headsign": "VALLEE BEREULT" if a.direction_id == "0" else "PARC JARDIN",
                      "trip_short_name": "", "direction_id": a.direction_id,
                      "wheelchair_accessible": "", "bikes_allowed": ""})
        for seq, (i, t) in enumerate(sequence, start=1):
            stop_times.append({"trip_id": trip_id, "arrival_time": hms(start + t), "departure_time": hms(start + t),
                               "stop_id": f"C{i}", "stop_sequence": seq, "pickup_type": 0, "drop_off_type": 0})

    GTFS_LIGNE_C_DIR.mkdir(exist_ok=True)
    stops.to_csv(GTFS_LIGNE_C_DIR / "stops.txt", index=False)
    routes.to_csv(GTFS_LIGNE_C_DIR / "routes.txt", index=False)
    pd.DataFrame(trips).to_csv(GTFS_LIGNE_C_DIR / "trips.txt", index=False)
    pd.DataFrame(stop_times).to_csv(GTFS_LIGNE_C_DIR / "stop_times.txt", index=False)
    print(f"Ligne C : {len(stops)} arrêts, {len(trips)} courses -> {GTFS_LIGNE_C_DIR}")


if __name__ == "__main__":
    main()
