"""Meilleurs trajets en transports en commun + marche pour les requêtes d'instances lehavre_rural.

Pour chaque requête, calcule l'ensemble de Pareto des trajets respectant sa fenêtre de temps (le MRT
n'est pas imposé), puis retient le meilleur selon trois critères :
  temps     : temps porte à porte minimal (puis correspondances, puis marche)
  marche    : temps de marche minimal (puis temps, puis correspondances)
  transferts: nombre de correspondances minimal (puis temps, puis marche)

Exemple : python resoudre_tc.py -t scolaire -i 30_30_0 30_30_1
"""
import argparse
import time

import pandas as pd

from config_tc import DAY_TYPES, INSTANCES_DIR, MAX_ACCESS_WALK, RESULTS_DIR
from mcraptor import McRaptor
from reseau import Timetable, WalkNetwork

CRITERIA = {
    "temps": lambda j: (j.travel, j.transfers, j.walk),
    "marche": lambda j: (j.walk, j.travel, j.transfers),
    "transferts": lambda j: (j.transfers, j.travel, j.walk),
}


def hhmm(minutes):
    m = int(round(minutes))
    return f"{m // 60:02d}:{m % 60:02d}"


def describe(journey):
    parts = []
    for mode, line, a, b, t0, t1 in journey.legs:
        if mode == "marche":
            if t1 - t0 >= 0.5:
                parts.append(f"marche {t1 - t0:.0f} min")
        elif mode == "attente":
            parts.append(f"attente {t1 - t0:.0f} min")
        else:
            parts.append(f"[{line}] {a} {hhmm(t0)} -> {b} {hhmm(t1)}")
    return " | ".join(parts)


def to_minutes(hh_mm):
    h, m = hh_mm.split(":")
    return int(h) * 60 + int(m)


def solve_instance(name, solver, walk_net):
    reqs = pd.read_csv(INSTANCES_DIR / f"r{name}.csv")
    rows = []
    for q in reqs.itertuples():
        o = walk_net.node(q.pickup_lon, q.pickup_lat)
        d = walk_net.node(q.delivery_lon, q.delivery_lat)
        journeys = solver.solve(walk_net.to_stops(o, MAX_ACCESS_WALK), walk_net.to_stops(d, MAX_ACCESS_WALK),
                                q.type_fenetre, to_minutes(q.heure_reference), walk_net.between(o, d))
        for crit, key in CRITERIA.items():
            row = {"instance": name, "requete": q.requete, "critere": crit, "sens": q.sens,
                   "type_fenetre": q.type_fenetre, "heure_reference": q.heure_reference,
                   "temps_direct_voiture": q.temps_direct_voiture, "nb_trajets_pareto": len(journeys),
                   "faisable": bool(journeys)}
            if journeys:
                j = min(journeys, key=key)
                row.update(depart=hhmm(j.dep), arrivee=hhmm(j.arr), temps_trajet=round(j.travel, 1),
                           ratio_direct=round(j.travel / q.temps_direct_voiture, 2), marche=round(j.walk, 1),
                           correspondances=j.transfers, nb_vehicules=j.nb_vehicles, itineraire=describe(j))
            rows.append(row)
    return pd.DataFrame(rows)


def statistics(df):
    ok = df[df.faisable]
    stats = ok.groupby("critere", sort=False)[["temps_trajet", "ratio_direct", "marche", "correspondances"]] \
        .agg(["mean", "median", "min", "max"]).round(2)
    counts = df.groupby("critere", sort=False).agg(requetes=("faisable", "size"), faisables=("faisable", "sum"))
    marche_seule = ok[ok.nb_vehicules == 0].groupby("critere", sort=False).size().rename("marche_seule")
    return counts.join(marche_seule).fillna(0).astype(int), stats


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("-t", "--type", required=True, choices=list(DAY_TYPES))
    parser.add_argument("-i", "--instances", nargs="*",
                        help="noms d'instances (ex. 30_30_0) ; par défaut toutes celles du dossier")
    args = parser.parse_args()
    names = args.instances or sorted(p.stem[1:] for p in INSTANCES_DIR.glob("r*.csv"))

    t0 = time.time()
    tt = Timetable(DAY_TYPES[args.type])
    walk_net = WalkNetwork(tt.stops)
    solver = McRaptor(tt, walk_net)
    print(f"Réseau du {DAY_TYPES[args.type]} ({args.type}) : {len(tt.patterns)} motifs de lignes "
          f"({time.time() - t0:.0f} s)")

    out = RESULTS_DIR / args.type
    out.mkdir(parents=True, exist_ok=True)
    all_df = []
    for name in names:
        t0 = time.time()
        df = solve_instance(name, solver, walk_net)
        df.to_csv(out / f"itineraires_{name}.csv", index=False)
        all_df.append(df)
        print(f"  {name} : {df[df.critere == 'temps'].faisable.sum()}/{df.requete.nunique()} requêtes faisables "
              f"({time.time() - t0:.0f} s)")

    df = pd.concat(all_df)
    counts, stats = statistics(df)
    label = names[0] if len(names) == 1 else f"{len(names)}_instances"
    stats.to_csv(out / f"statistiques_{label}.csv")
    pd.set_option("display.width", 200)
    print(f"\n=== {args.type} : {len(names)} instance(s), {df.requete.size // len(CRITERIA)} requêtes ===")
    print(counts.to_string())
    print()
    print(stats.to_string())
    print(f"\nRésultats dans {out}")


if __name__ == "__main__":
    main()
