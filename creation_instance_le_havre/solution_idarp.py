"""Lecture et vérification des solutions écrites par idarp_algo (main_LNS -sol, voir Solution::write).

  .venv/bin/python solution_idarp.py <fichier solution> [dossier des instances]

La vérification n'utilise que les fichiers de l'instance (requêtes, matrices, courses de tram) : temps de
conduite entre arrêts consécutifs, capacité, fenêtres, temps de trajet maximal, marche, et existence de
chaque course de tram (mode -tram) avec ses heures de passage.
"""
import json
import sys
from pathlib import Path

import networkx as nx
import numpy as np
import osmnx as ox
import pandas as pd

from config import INSTANCES_DIR, TIME_ORIGIN

ROLES = {"P": "pickup", "D": "delivery", "SB": "depose_tc", "SA": "reprise_tc"}


def read(path):
    """Solution : en-tête (instance, pt, seed, objective), tournées et motif de chaque requête."""
    lines = iter(Path(path).read_text().split("\n"))
    sol = {"routes": [], "requests": {}}
    for line in lines:
        key, *rest = line.split()
        if key == "routes":
            break
        sol[key] = rest if len(rest) > 1 else rest[0]
    sol["objective"] = int(sol["objective"])
    nb_routes = int(rest[0])
    for _ in range(nb_routes):
        _, r, n = next(lines).split()
        stops = []
        for _ in range(int(n)):
            role, req, point, time = next(lines).split()
            stops.append({"role": ROLES[role], "requete": int(req), "point": int(point), "temps": int(time)})
        sol["routes"].append(stops)
    _, nb_requests = next(lines).split()
    for _ in range(int(nb_requests)):
        fields = next(lines).split()
        req, pattern, values = int(fields[0]), fields[1], list(map(int, fields[2:]))
        names = {"direct": ["route"],
                 "walk_pickup": ["route", "arret_proche", "marche", "depart_a_pied", "tc_depart", "tc_arrivee",
                                 "arret_vehicule"],
                 "walk_delivery": ["route", "arret_vehicule", "tc_depart", "tc_arrivee", "arret_proche", "marche",
                                   "arrivee_a_pied"],
                 "transfer": ["route_avant", "route_apres", "arret_depose", "arret_reprise", "tc_depart",
                              "tc_arrivee"],
                 "unassigned": []}[pattern]
        sol["requests"][req] = {"motif": pattern, **dict(zip(names, values))}
    return sol


def tram_rides(path, first_stop):
    """Ensemble des trajets directs (point départ, point arrivée, heure départ, heure arrivée) des courses."""
    rides = set()
    lines = Path(path).read_text().split("\n")
    for line in lines[1:int(lines[0]) + 1]:
        f = line.split()
        stops = [(first_stop + int(f[2 + 2 * k]), int(f[3 + 2 * k])) for k in range(int(f[1]))]
        for i in range(len(stops)):
            for j in range(i + 1, len(stops)):
                rides.add((stops[i][0], stops[j][0], stops[i][1], stops[j][1]))
    return rides


def verify(sol, folder=INSTANCES_DIR):
    """Liste des violations (vide si la solution est réalisable)."""
    folder = Path(folder)
    name = sol["instance"][1:-4]   # i30_30_0.txt -> 30_30_0
    header, *reqs = (folder / sol["instance"]).read_text().strip().split("\n")
    nb_req, _, nb_stops, capacity, horizon = map(int, header.split())
    reqs = [list(map(int, r.split())) for r in reqs]
    drive = np.loadtxt(folder / f"d{name}.txt", dtype=int)
    walk = np.loadtxt(folder / f"w{name}.txt", dtype=int)
    first_stop = 1 + 2 * nb_req
    timetable = sol["pt"][0] == "tram"
    rides = tram_rides(folder / sol["pt"][1], first_stop) if timetable else None
    errors = []

    for r, stops in enumerate(sol["routes"]):
        prev, t_prev, service, load = 0, 0, 0, 0
        for s in stops:
            L, q = reqs[s["requete"]][8], reqs[s["requete"]][7]
            if s["temps"] < t_prev + service + drive[prev, s["point"]]:
                errors.append(f"route {r} : arrêt {s['point']} servi trop tôt")
            load += q if s["role"] in ("pickup", "reprise_tc") else -q
            if load > capacity:
                errors.append(f"route {r} : capacité dépassée")
            prev, t_prev, service = s["point"], s["temps"], L
        if stops and t_prev + service + drive[prev, 0] > horizon:
            errors.append(f"route {r} : retour au dépôt après l'horizon")

    times = {}
    for r, stops in enumerate(sol["routes"]):
        for s in stops:
            times[(s["requete"], s["role"])] = (r, s["point"], s["temps"])
    for i, (idp, a, b, idd, c, d, mrt, q, L) in enumerate(reqs):
        x = sol["requests"][i]
        m = x["motif"]

        def check(label, t, lo, hi):
            if not lo <= t <= hi:
                errors.append(f"requête {i} ({m}) : {label} {t} hors de [{lo}, {hi}]")

        def ride(dep_stop, arr_stop, t_dep, t_arr):
            if timetable and (dep_stop, arr_stop, t_dep, t_arr) not in rides:
                errors.append(f"requête {i} : aucune course de tram {dep_stop}->{arr_stop} à {t_dep}->{t_arr}")

        if m == "unassigned":
            errors.append(f"requête {i} non servie")
        elif m == "direct":
            tp, td = times[(i, "pickup")][2], times[(i, "delivery")][2]
            check("pickup", tp, a, b), check("delivery", td, c, d), check("temps de trajet", td - tp, 0, mrt)
        elif m == "walk_pickup":
            ts, td = times[(i, "reprise_tc")][2], times[(i, "delivery")][2]
            if walk[idp, x["arret_proche"]] != x["marche"]:
                errors.append(f"requête {i} : marche {x['marche']} au lieu de {walk[idp, x['arret_proche']]}")
            check("départ à pied", x["depart_a_pied"], a, b)
            if x["depart_a_pied"] + x["marche"] > x["tc_depart"] or x["tc_arrivee"] > ts:
                errors.append(f"requête {i} : correspondance marche/tram/véhicule impossible")
            ride(x["arret_proche"], x["arret_vehicule"], x["tc_depart"], x["tc_arrivee"])
            check("delivery", td, c, d), check("temps de trajet", td - x["depart_a_pied"], 0, mrt)
        elif m == "walk_delivery":
            tp, ts = times[(i, "pickup")][2], times[(i, "depose_tc")][2]
            if walk[idd, x["arret_proche"]] != x["marche"]:
                errors.append(f"requête {i} : marche {x['marche']} au lieu de {walk[idd, x['arret_proche']]}")
            if ts + L > x["tc_depart"] or x["tc_arrivee"] + x["marche"] != x["arrivee_a_pied"]:
                errors.append(f"requête {i} : correspondance véhicule/tram/marche impossible")
            ride(x["arret_vehicule"], x["arret_proche"], x["tc_depart"], x["tc_arrivee"])
            check("pickup", tp, a, b), check("arrivée à pied", x["arrivee_a_pied"], c, d)
            check("temps de trajet", x["arrivee_a_pied"] - tp, 0, mrt)
        elif m == "transfer":
            tp, tsb = times[(i, "pickup")][2], times[(i, "depose_tc")][2]
            tsa, td = times[(i, "reprise_tc")][2], times[(i, "delivery")][2]
            if tsb + L > x["tc_depart"] or x["tc_arrivee"] > tsa:
                errors.append(f"requête {i} : correspondance véhicule/tram/véhicule impossible")
            ride(x["arret_depose"], x["arret_reprise"], x["tc_depart"], x["tc_arrivee"])
            check("pickup", tp, a, b), check("delivery", td, c, d), check("temps de trajet", td - tp, 0, mrt)
    return errors


def hhmm(t):
    m = int(t) + TIME_ORIGIN
    return f"{m // 60:02d}:{m % 60:02d}"


def trips_for_map(sol, walk_graph, folder=INSTANCES_DIR):
    """Solution sous la forme affichée par la carte (05_visualisation) : pour chaque requête son trajet
    (étapes véhicule / tram / marche avec horaires et tracés), et les tournées complètes des véhicules."""
    folder = Path(folder)
    name = sol["instance"][1:-4]
    points = gpd_points(folder / f"l{name}.geojson")
    desc = pd.read_csv(folder / f"r{name}.csv").set_index("requete")
    nb_req = len(desc)
    runs = []
    lines = (folder / sol["pt"][1].replace("public_transport_time_", "tram_runs_")).read_text().split("\n")
    for line in lines[1:int(lines[0]) + 1]:
        f = line.split()
        runs.append((f[0], [(1 + 2 * nb_req + int(f[2 + 2 * k]), int(f[3 + 2 * k])) for k in range(int(f[1]))]))

    def label(p):
        return points[p]["nom"] or ("dépôt" if p == 0 else "pickup" if p % 2 else "delivery")

    def tram(a, b, ta, tb):
        """Ligne et stations desservies : la course aux heures données (mode -tram), sinon une course a -> b."""
        best = None
        for line, stops in runs:
            pos = {s: k for k, (s, _) in enumerate(stops)}
            if a in pos and b in pos and pos[a] < pos[b]:
                path = stops[pos[a]:pos[b] + 1]
                if sol["pt"][0] != "tram" or (path[0][1] == ta and path[-1][1] == tb):
                    return line, [points[s]["ll"] for s, _ in path]
                best = best or (line, [points[s]["ll"] for s, _ in path])
        return best or ("?", [points[a]["ll"], points[b]["ll"]])

    def walk(a_ll, b_ll):
        u = ox.distance.nearest_nodes(walk_graph, a_ll[1], a_ll[0])
        v = ox.distance.nearest_nodes(walk_graph, b_ll[1], b_ll[0])
        path = nx.shortest_path(walk_graph, u, v, weight="length")
        return [a_ll] + [[walk_graph.nodes[n]["y"], walk_graph.nodes[n]["x"]] for n in path] + [b_ll]

    def vehicle(r, p_from, p_to):
        stops = sol["routes"][r]
        i = next(k for k, s in enumerate(stops) if s["point"] == p_from and s["requete"] in current)
        j = next(k for k in range(i + 1, len(stops)) if stops[k]["point"] == p_to and stops[k]["requete"] in current)
        return stops[i]["temps"], stops[j]["temps"], [points[s["point"]]["ll"] for s in stops[i:j + 1]]

    def leg(mode, line, a, b, t0, t1, trace, **extra):
        return {"mode": mode, "ligne": line, "de": a, "a": b, "depart": hhmm(t0), "arrivee": hhmm(t1),
                "trace": [[round(x, 6) for x in ll] for ll in trace], "_t": (t0, t1), **extra}

    out = {}
    for i, x in sol["requests"].items():
        current = {i}
        pick, dlv = 2 * i + 1, 2 * i + 2
        m, legs = x["motif"], []
        if m == "unassigned":
            out[i] = None
            continue
        if m == "direct":
            t0, t1, tr = vehicle(x["route"], pick, dlv)
            legs.append(leg("vehicule", f"véhicule {x['route']}", "pickup", "delivery", t0, t1, tr, vehicule=x["route"]))
        elif m == "walk_pickup":
            n, s = x["arret_proche"], x["arret_vehicule"]
            legs.append(leg("marche", "", "origine", label(n), x["depart_a_pied"], x["depart_a_pied"] + x["marche"],
                            walk(points[pick]["ll"], points[n]["ll"])))
            line, tr = tram(n, s, x["tc_depart"], x["tc_arrivee"])
            legs.append(leg("tram", line, label(n), label(s), x["tc_depart"], x["tc_arrivee"], tr))
            t0, t1, tr = vehicle(x["route"], s, dlv)
            legs.append(leg("vehicule", f"véhicule {x['route']}", label(s), "delivery", t0, t1, tr, vehicule=x["route"]))
        elif m == "walk_delivery":
            s, n = x["arret_vehicule"], x["arret_proche"]
            t0, t1, tr = vehicle(x["route"], pick, s)
            legs.append(leg("vehicule", f"véhicule {x['route']}", "pickup", label(s), t0, t1, tr, vehicule=x["route"]))
            line, tr = tram(s, n, x["tc_depart"], x["tc_arrivee"])
            legs.append(leg("tram", line, label(s), label(n), x["tc_depart"], x["tc_arrivee"], tr))
            legs.append(leg("marche", "", label(n), "destination", x["tc_arrivee"], x["arrivee_a_pied"],
                            walk(points[n]["ll"], points[dlv]["ll"])))
        else:  # transfer
            sb, sa = x["arret_depose"], x["arret_reprise"]
            t0, t1, tr = vehicle(x["route_avant"], pick, sb)
            legs.append(leg("vehicule", f"véhicule {x['route_avant']}", "pickup", label(sb), t0, t1, tr,
                            vehicule=x["route_avant"]))
            line, tr = tram(sb, sa, x["tc_depart"], x["tc_arrivee"])
            legs.append(leg("tram", line, label(sb), label(sa), x["tc_depart"], x["tc_arrivee"], tr))
            t0, t1, tr = vehicle(x["route_apres"], sa, dlv)
            legs.append(leg("vehicule", f"véhicule {x['route_apres']}", label(sa), "delivery", t0, t1, tr,
                            vehicule=x["route_apres"]))
        dep, arr = legs[0]["_t"][0], legs[-1]["_t"][1]
        for l in legs:
            del l["_t"]
        direct = int(desc.temps_direct_voiture[i])
        out[i] = {"motif": {"direct": "direct", "walk_pickup": "marche + tram + véhicule",
                            "walk_delivery": "véhicule + tram + marche",
                            "transfer": "véhicule + tram + véhicule"}[m],
                  "depart": hhmm(dep), "arrivee": hhmm(arr), "temps_trajet": arr - dep,
                  "ratio_direct": round((arr - dep) / direct, 2), "marche": x.get("marche", 0),
                  "correspondances": len(legs) - 1, "etapes": legs}
    roles = {"pickup": "pickup", "delivery": "delivery", "depose_tc": "dépose au tram", "reprise_tc": "reprise au tram"}
    routes = [{"vehicule": r,
               "trace": [points[0]["ll"]] + [points[s["point"]]["ll"] for s in stops] + [points[0]["ll"]],
               "arrets": [{"ll": points[s["point"]]["ll"], "role": roles[s["role"]], "requete": s["requete"],
                           "lieu": label(s["point"]), "heure": hhmm(s["temps"])} for s in stops]}
              for r, stops in enumerate(sol["routes"]) if stops]
    return {"requetes": out, "routes": routes, "objectif": sol["objective"]}


def gpd_points(path):
    """Points d'une instance (l*.geojson) : index -> {"ll": [lat, lon], "nom": nom de station ou None}."""
    feats = json.loads(Path(path).read_text())["features"]
    return {f["properties"]["index"]: {"ll": [f["properties"]["lat"], f["properties"]["lon"]],
                                       "nom": f["properties"].get("nom")} for f in feats}


def main():
    sol = read(sys.argv[1])
    errors = verify(sol, sys.argv[2] if len(sys.argv) > 2 else INSTANCES_DIR)
    motifs = {}
    for x in sol["requests"].values():
        motifs[x["motif"]] = motifs.get(x["motif"], 0) + 1
    print(f"{sol['instance']} ({' '.join(sol['pt'])}) : objectif {sol['objective']}, {motifs}")
    print("\n".join(errors) if errors else "Aucune violation.")


if __name__ == "__main__":
    main()
