"""Prépare les données nécessaires à la génération des instances (data/prepared/).

- réseaux piéton et routier OSM de la Communauté urbaine (osmnx) ;
- stations de tram (GTFS LiA, arrêts regroupés par nom) ;
- temps de marche de chaque nœud piéton à la station de tram la plus proche ;
- points "ville" : nœuds piétons à moins de NEAR_TRAM_MAX_WALK min d'une station ;
- points "rural" : nœuds piétons à plus de FAR_TRAM_MIN_WALK min de toute station,
  dans un carreau Filosofi peuplé situé à moins de MAX_DRIVE_FROM_CENTER min en
  voiture de l'Hôtel de Ville. Chaque carreau est pondéré par
  flux MOBPRO(commune -> Le Havre) * population du carreau / population de la commune.
"""
import glob

import geopandas as gpd
import networkx as nx
import numpy as np
import osmium
import osmnx as ox
import pandas as pd

from config import (CODE_LE_HAVRE, FAR_TRAM_MIN_WALK, GTFS_DIR, HOTEL_DE_VILLE, MAX_DRIVE_FROM_CENTER,
                    NEAR_TRAM_MAX_WALK, PREP_DIR, RAW_DIR, WALK_SPEED_KMH)
from temps_voiture import set_realistic_travel_times

L93 = "EPSG:2154"
WALK_M_PER_MIN = WALK_SPEED_KMH * 1000 / 60


def load_communes():
    communes = gpd.read_file(RAW_DIR / "communes_cu.geojson").to_crs(L93)
    return communes[["code", "nom", "population", "geometry"]]


# Filtres repris de ceux d'osmnx pour network_type="drive" / "walk".
DRIVE_HIGHWAYS = {"motorway", "motorway_link", "trunk", "trunk_link", "primary", "primary_link", "secondary",
                  "secondary_link", "tertiary", "tertiary_link", "unclassified", "residential", "living_street",
                  "road"}
NO_WALK_HIGHWAYS = {"motorway", "motorway_link", "trunk", "trunk_link", "abandoned", "bus_guideway",
                    "construction", "cycleway", "platform", "proposed", "raceway", "bus_stop", "elevator"}


def is_drive_way(tags):
    return (tags.get("highway") in DRIVE_HIGHWAYS and tags.get("area") != "yes"
            and tags.get("access") not in ("private", "no") and tags.get("motor_vehicle") != "no"
            and tags.get("motorcar") != "no")


def is_walk_way(tags):
    return (tags.get("highway") not in NO_WALK_HIGHWAYS and tags.get("area") != "yes"
            and tags.get("foot") != "no" and tags.get("access") != "private"
            and tags.get("service") != "private")


def extract_osm(bounds, accept, out_file):
    """Écrit en XML les voies de l'extrait PBF acceptées par `accept` et touchant `bounds`."""
    xmin, ymin, xmax, ymax = bounds
    pbf = RAW_DIR / "haute-normandie.osm.pbf"
    fp = (osmium.FileProcessor(pbf).with_locations()
          .with_filter(osmium.filter.EntityFilter(osmium.osm.WAY))
          .with_filter(osmium.filter.KeyFilter("highway")))
    with osmium.BackReferenceWriter(str(out_file), ref_src=pbf, overwrite=True) as writer:
        for way in fp:
            tags = dict(way.tags)
            if accept(tags) and any(xmin <= n.lon <= xmax and ymin <= n.lat <= ymax
                                    for n in way.nodes if n.location.valid()):
                writer.add(way)


def load_graphs(territory_wgs84):
    walk_file, drive_file = PREP_DIR / "walk.graphml", PREP_DIR / "drive.graphml"
    if walk_file.exists() and drive_file.exists():
        return ox.load_graphml(walk_file), ox.load_graphml(drive_file)
    print("Construction des réseaux à partir de l'extrait OSM...")
    bounds = territory_wgs84.bounds
    extract_osm(bounds, is_walk_way, PREP_DIR / "walk.osm")
    extract_osm(bounds, is_drive_way, PREP_DIR / "drive.osm")
    walk = ox.graph_from_xml(PREP_DIR / "walk.osm", bidirectional=True, retain_all=False)
    drive = ox.graph_from_xml(PREP_DIR / "drive.osm", bidirectional=False, retain_all=True)
    walk = ox.truncate.truncate_graph_polygon(walk, territory_wgs84)
    walk = ox.truncate.largest_component(walk)
    drive = ox.truncate.truncate_graph_polygon(drive, territory_wgs84)
    drive = ox.truncate.largest_component(drive, strongly=True)
    drive = ox.add_edge_travel_times(ox.add_edge_speeds(drive))
    ox.save_graphml(walk, walk_file)
    ox.save_graphml(drive, drive_file)
    return walk, drive


def tram_stations():
    routes = pd.read_csv(GTFS_DIR / "routes.txt")
    trips = pd.read_csv(GTFS_DIR / "trips.txt")
    stop_times = pd.read_csv(GTFS_DIR / "stop_times.txt", dtype={"stop_id": str})
    stops = pd.read_csv(GTFS_DIR / "stops.txt", dtype={"stop_id": str})
    tram_trips = trips[trips.route_id.isin(routes[routes.route_type == 0].route_id)]
    tram_stop_ids = stop_times[stop_times.trip_id.isin(tram_trips.trip_id)].stop_id.unique()
    tram_stops = stops[stops.stop_id.isin(tram_stop_ids)]
    stations = (tram_stops.groupby("stop_name")
                .agg(stop_ids=("stop_id", lambda s: ";".join(sorted(s))),
                     lat=("stop_lat", "mean"), lon=("stop_lon", "mean"))
                .reset_index().rename(columns={"stop_name": "name"}))
    stations["station"] = range(len(stations))
    return gpd.GeoDataFrame(stations, geometry=gpd.points_from_xy(stations.lon, stations.lat), crs="EPSG:4326")


def walk_time_to_tram(walk, stations):
    """Temps de marche (min) de chaque nœud piéton vers la station la plus proche, et laquelle."""
    undirected = walk.to_undirected()
    station_nodes = ox.distance.nearest_nodes(walk, stations.lon, stations.lat)
    best = {}
    for st, node in zip(stations.station, station_nodes):
        for n, d in nx.single_source_dijkstra_path_length(undirected, node, weight="length").items():
            if n not in best or d < best[n][0]:
                best[n] = (d, st)
    nodes = ox.graph_to_gdfs(walk, edges=False)[["x", "y"]]
    nodes["walk_to_tram"] = [best.get(n, (np.inf, -1))[0] / WALK_M_PER_MIN for n in nodes.index]
    nodes["nearest_station"] = [best.get(n, (np.inf, -1))[1] for n in nodes.index]
    return nodes


def drive_time_from_center(drive):
    center = ox.distance.nearest_nodes(drive, *HOTEL_DE_VILLE)
    times = nx.single_source_dijkstra_path_length(drive, center, weight="travel_time")
    nodes = ox.graph_to_gdfs(drive, edges=False)[["x", "y"]]
    nodes["drive_from_center"] = [times.get(n, np.inf) / 60 for n in nodes.index]
    return nodes


def load_carreaux(communes):
    gpkg = RAW_DIR / "filosofi_200m" / "carreaux_200m_met.gpkg"
    car = gpd.read_file(gpkg, bbox=tuple(communes.total_bounds), columns=["idcar_200m", "ind"])  # Lambert 93
    # Un carreau à cheval sur deux communes est attribué à celle qui contient son centre.
    centers = gpd.GeoDataFrame(car[["idcar_200m"]], geometry=car.geometry.centroid, crs=car.crs)
    centers = gpd.sjoin(centers, communes[["code", "geometry"]], predicate="within", how="inner")
    return car.merge(centers[["idcar_200m", "code"]], on="idcar_200m")


def mobpro_weights(communes):
    csv = glob.glob(str(RAW_DIR / "mobpro" / "*.csv"))[0]
    flux = pd.read_csv(csv, sep=";", dtype={"CODGEO": str, "DCLT": str})
    col = [c for c in flux.columns if c.startswith("NBFLUX")][0]
    to_lh = flux[(flux.DCLT == CODE_LE_HAVRE) & flux.CODGEO.isin(communes.code)]
    return to_lh.set_index("CODGEO")[col].rename("flux_vers_le_havre")


def main():
    PREP_DIR.mkdir(parents=True, exist_ok=True)
    communes = load_communes()
    territory = communes.to_crs("EPSG:4326").union_all().buffer(0.01)
    walk, drive = load_graphs(territory)
    set_realistic_travel_times(drive)
    print(f"Réseau piéton : {len(walk)} nœuds, routier : {len(drive)} nœuds")

    stations = tram_stations()
    stations.to_file(PREP_DIR / "tram_stations.geojson")

    nodes = walk_time_to_tram(walk, stations)
    drive_nodes = drive_time_from_center(drive)
    nodes["drive_node"] = ox.distance.nearest_nodes(drive, nodes.x, nodes.y)
    nodes["drive_from_center"] = drive_nodes.drive_from_center.reindex(nodes.drive_node).to_numpy()
    nodes = gpd.GeoDataFrame(nodes, geometry=gpd.points_from_xy(nodes.x, nodes.y), crs="EPSG:4326")
    nodes = gpd.sjoin(nodes.to_crs(L93), communes[["code", "geometry"]], predicate="within", how="inner")
    nodes = nodes.drop(columns="index_right", errors="ignore")

    # --- Points "ville" ---
    city = nodes[np.rint(nodes.walk_to_tram) <= NEAR_TRAM_MAX_WALK]  # arrondi comme dans les matrices
    city.reset_index()[["osmid", "x", "y", "walk_to_tram", "nearest_station", "drive_node", "code"]] \
        .to_csv(PREP_DIR / "city_nodes.csv", index=False)

    # --- Points "rural" ---
    carreaux = load_carreaux(communes)
    rural = nodes[(np.rint(nodes.walk_to_tram) > FAR_TRAM_MIN_WALK) & (nodes.drive_from_center <= MAX_DRIVE_FROM_CENTER)]
    rural = gpd.sjoin(rural, carreaux[["idcar_200m", "ind", "geometry"]], predicate="within", how="inner")
    rural = rural.drop(columns="index_right", errors="ignore")

    flux = mobpro_weights(communes)
    pop = communes.set_index("code").population
    car_w = (carreaux[carreaux.idcar_200m.isin(rural.idcar_200m)]
             .assign(flux=lambda c: c.code.map(flux).fillna(0), pop_commune=lambda c: c.code.map(pop)))
    car_w["weight"] = car_w.flux * car_w.ind / car_w.pop_commune
    car_w = car_w[car_w.weight > 0]
    rural = rural[rural.idcar_200m.isin(car_w.idcar_200m)]

    rural.reset_index()[["osmid", "x", "y", "walk_to_tram", "drive_node", "drive_from_center", "code", "idcar_200m"]] \
        .to_csv(PREP_DIR / "rural_nodes.csv", index=False)
    car_w.to_file(PREP_DIR / "carreaux_ruraux.geojson")

    summary = (car_w.groupby("code").agg(carreaux=("idcar_200m", "size"), pop_eligible=("ind", "sum"),
                                         flux=("flux", "first"), poids=("weight", "sum"))
               .join(communes.set_index("code").nom).sort_values("poids", ascending=False))
    summary["part_requetes_%"] = (100 * summary.poids / summary.poids.sum()).round(1)
    summary.to_csv(PREP_DIR / "communes_origine.csv")
    print(summary.to_string())
    print(f"{len(city)} nœuds ville, {len(rural)} nœuds ruraux dans {len(car_w)} carreaux")


if __name__ == "__main__":
    main()
