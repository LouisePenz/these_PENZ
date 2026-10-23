"""Compte, pour chaque requête, les modes possibles selon les règles de idarp_algo (Instance.cpp) :

- marche + tram + véhicule (ou l'inverse) : computeWalkData ;
- véhicule + tram + véhicule : computeStopPairs ;
- sinon trajet direct obligatoire.

Les paramètres waiting_time, minimum_time_in_transport_public et threshold_to_walk sont lus dans
idarp_algo/config_instance.txt.
"""
import argparse

import numpy as np
import pandas as pd

from config import DAYS, INSTANCES_DIR, ROOT


def read_conf(path):
    conf = dict(line.strip().split(":") for line in open(path) if ":" in line)
    return int(conf["waiting_time"]), int(conf["minimum_time_in_transport_public"]), int(conf["threshold_to_walk"])


def read_requests(path):
    with open(path) as f:
        n = int(f.readline().split()[0])
        return n, [list(map(int, f.readline().split())) for _ in range(n)]


def options(req, drive, walk, pt, first_stop, waiting, min_transit, threshold):
    p, ltw_p, utw_p, d, ltw_d, utw_d, mrt, _, L = req
    stops = range(first_stop, first_stop + pt.shape[0])
    ptt = lambda a, b: pt[a - first_stop, b - first_stop] + waiting

    # Marche à une extrémité (l'extrémité "ville")
    walk_mode = None
    near_p = min(stops, key=lambda s: walk[p, s])
    near_d = min(stops, key=lambda s: walk[d, s])
    if walk[p, near_p] <= threshold:
        w = walk[p, near_p]
        for s in stops:
            if w + ptt(near_p, s) + L + drive[s, d] <= mrt and \
                    ltw_p + w + ptt(near_p, s) <= min(utw_p + w + ptt(near_p, s), utw_d - L - drive[s, d]):
                walk_mode = "marche-tram-véhicule" if s != near_p else "marche-véhicule (même arrêt)"
                if s != near_p:
                    break
    elif walk[d, near_d] <= threshold:
        w = walk[d, near_d]
        for s in stops:
            if L + drive[p, s] + L + ptt(s, near_d) + w <= mrt and \
                    max(ltw_p + L + drive[p, s], ltw_d - L - ptt(s, near_d) - w) <= utw_d - L - ptt(s, near_d) - w:
                walk_mode = "véhicule-tram-marche" if s != near_d else "véhicule-marche (même arrêt)"
                if s != near_d:
                    break

    # Deux véhicules reliés par le tram
    two_vehicles = any(
        drive[p, sb] + ptt(sb, sa) + drive[sa, d] + 3 * L <= mrt and ptt(sb, sa) >= min_transit
        and ltw_p + L + drive[p, sb] <= utw_d - L - drive[sa, d] - L - ptt(sb, sa)
        for sb in stops for sa in stops)
    return walk_mode, two_vehicles


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("-n", "--nb-requetes", type=int, default=30)
    parser.add_argument("-c", "--config", default=str(ROOT.parent / "idarp_algo" / "config_instance.txt"))
    args = parser.parse_args()
    waiting, min_transit, threshold = read_conf(args.config)
    n = args.nb_requetes

    rows = []
    for i_file in sorted(INSTANCES_DIR.glob(f"i{n}_{n}_*.txt")):
        name = i_file.stem[1:]
        _, reqs = read_requests(i_file)
        drive = np.loadtxt(INSTANCES_DIR / f"d{name}.txt", dtype=int)
        walk = np.loadtxt(INSTANCES_DIR / f"w{name}.txt", dtype=int)
        for day in DAYS:
            pt = np.loadtxt(INSTANCES_DIR / f"public_transport_time_{day}.txt", dtype=int)
            for r, req in enumerate(reqs):
                walk_mode, two = options(req, drive, walk, pt, 1 + 2 * n, waiting, min_transit, threshold)
                rows.append({"instance": name, "jour": day, "requete": r,
                             "marche": walk_mode or "non", "deux_vehicules": two})
    df = pd.DataFrame(rows)
    df["direct_obligatoire"] = (df.marche == "non") & ~df.deux_vehicules
    df.to_csv(INSTANCES_DIR / f"options_{n}.csv", index=False)

    print(f"waiting_time={waiting}, minimum_time_in_transport_public={min_transit}, threshold_to_walk={threshold}\n")
    for day, g in df.groupby("jour", sort=False):
        print(f"== {day} ({len(g)} requêtes)")
        print(g.marche.value_counts().to_string())
        print(f"deux véhicules + tram possible : {g.deux_vehicules.sum()}")
        print(f"direct obligatoire : {g.direct_obligatoire.sum()}\n")
    per_inst = df[df.jour == "lundi_scolaire"].groupby("instance").agg(
        marche_tram=("marche", lambda m: m.str.contains("tram").sum()), direct=("direct_obligatoire", "sum"))
    print("Par instance (lundi scolaire) :\n" + per_inst.to_string())


if __name__ == "__main__":
    main()
