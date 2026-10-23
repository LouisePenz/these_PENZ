"""Chargement du réseau : horaires GTFS d'un jour et réseau piéton OSM.

Les temps de marche sont arrondis à la minute, comme les horaires GTFS.
"""
from dataclasses import dataclass

import numpy as np
import osmnx as ox
import pandas as pd
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

from config_tc import (CACHE_DIR, GTFS_DIR, MAX_STOP_SNAP, MAX_TRANSFER_WALK, WALK_GRAPH, WALK_SPEED_KMH)

WALK_M_PER_MIN = WALK_SPEED_KMH * 1000 / 60


@dataclass
class Pattern:
    """Ensemble de courses d'une même ligne desservant la même suite d'arrêts, triées par départ."""
    route: str                # nom court de la ligne (A, B, 09, C1...)
    stops: list               # indices des arrêts
    arr: np.ndarray           # (courses x arrêts), minutes depuis minuit
    dep: np.ndarray
    trip_ids: list


class Timetable:
    def __init__(self, day):
        yyyymmdd = day.replace("-", "")
        cal = pd.read_csv(GTFS_DIR / "calendar_dates.txt", dtype=str)
        services = set(cal[(cal.date == yyyymmdd) & (cal.exception_type == "1")].service_id)
        routes = pd.read_csv(GTFS_DIR / "routes.txt", dtype=str).set_index("route_id")
        trips = pd.read_csv(GTFS_DIR / "trips.txt", dtype=str)
        trips = trips[trips.service_id.isin(services)].set_index("trip_id")
        self.stops = pd.read_csv(GTFS_DIR / "stops.txt", dtype={"stop_id": str})
        stop_index = {sid: i for i, sid in enumerate(self.stops.stop_id)}

        st = pd.read_csv(GTFS_DIR / "stop_times.txt", dtype={"stop_id": str, "trip_id": str})
        st = st[st.trip_id.isin(trips.index)].sort_values(["trip_id", "stop_sequence"])
        to_min = lambda s: s.str.split(":").map(lambda x: int(x[0]) * 60 + int(x[1]) + int(x[2]) / 60)
        st["arr"], st["dep"] = to_min(st.arrival_time), to_min(st.departure_time)
        st["stop"] = st.stop_id.map(stop_index)

        groups = {}
        for trip_id, t in st.groupby("trip_id", sort=False):
            key = (trips.route_id[trip_id], tuple(t.stop))
            groups.setdefault(key, []).append((t.dep.iloc[0], trip_id, t.arr.to_numpy(), t.dep.to_numpy()))
        self.patterns = []
        for (route_id, stops), ts in groups.items():
            ts.sort(key=lambda x: x[0])
            self.patterns.append(Pattern(routes.route_short_name[route_id], list(stops),
                                         np.array([x[2] for x in ts]), np.array([x[3] for x in ts]),
                                         [x[1] for x in ts]))
        # arrêt -> [(motif, position)]
        self.stop_patterns = [[] for _ in range(len(self.stops))]
        for p_i, p in enumerate(self.patterns):
            for pos, s in enumerate(p.stops):
                self.stop_patterns[s].append((p_i, pos))

    def stop_name(self, s):
        return self.stops.stop_name.iloc[s]


class WalkNetwork:
    def __init__(self, stops):
        g = ox.load_graphml(WALK_GRAPH)
        self.graph = g
        self.index = {n: i for i, n in enumerate(g.nodes)}
        edges = pd.DataFrame([(self.index[u], self.index[v], float(d["length"]) / WALK_M_PER_MIN)
                              for u, v, d in g.edges(data=True)], columns=["u", "v", "t"])
        edges = edges.groupby(["u", "v"]).t.min().reset_index()
        self.matrix = csr_matrix((edges.t, (edges.u, edges.v)), shape=(len(self.index),) * 2)

        nodes, dist = ox.distance.nearest_nodes(g, stops.stop_lon, stops.stop_lat, return_dist=True)
        self.stop_node = np.array([self.index[n] for n in nodes])
        self.stop_ok = np.asarray(dist) <= MAX_STOP_SNAP     # arrêts reliés au réseau piéton
        self.stop_snap = np.asarray(dist) / WALK_M_PER_MIN   # marche entre l'arrêt et son nœud
        self.footpaths = self._footpaths()

    def node(self, lon, lat):
        return self.index[ox.distance.nearest_nodes(self.graph, lon, lat)]

    def to_stops(self, node, limit):
        """Temps de marche (min, arrondi) d'un nœud vers chaque arrêt relié, si <= limit : {arrêt: temps}."""
        t = dijkstra(self.matrix, directed=False, indices=node, limit=limit)[self.stop_node] + self.stop_snap
        return {s: float(round(t[s])) for s in np.flatnonzero((t <= limit) & self.stop_ok)}

    def between(self, a, b):
        return float(round(dijkstra(self.matrix, directed=False, indices=a)[b]))

    def _footpaths(self):
        """Correspondances à pied entre arrêts (<= MAX_TRANSFER_WALK min), mises en cache."""
        cache = CACHE_DIR / f"footpaths_{MAX_TRANSFER_WALK}.csv"
        if cache.exists():
            fp = pd.read_csv(cache)
        else:
            CACHE_DIR.mkdir(exist_ok=True)
            ok = np.flatnonzero(self.stop_ok)
            rows = []
            for chunk in np.array_split(ok, max(1, len(ok) // 100)):
                t = dijkstra(self.matrix, directed=False, indices=self.stop_node[chunk], limit=MAX_TRANSFER_WALK)
                t = t[:, self.stop_node[ok]] + self.stop_snap[chunk, None] + self.stop_snap[None, ok]
                for i, j in zip(*np.nonzero(t <= MAX_TRANSFER_WALK)):
                    if chunk[i] != ok[j]:
                        rows.append((chunk[i], ok[j], t[i, j]))
            fp = pd.DataFrame(rows, columns=["a", "b", "t"])
            fp.to_csv(cache, index=False)
        footpaths = [[] for _ in range(len(self.stop_node))]
        for a, b, t in fp.itertuples(index=False):
            footpaths[int(a)].append((int(b), float(round(t))))
        return footpaths
