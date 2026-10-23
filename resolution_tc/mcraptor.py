"""McRAPTOR (Delling et al., 2012) adapté à une fenêtre de départ ou d'arrivée.

Chaque étiquette porte trois critères : heure de départ du domicile (à maximiser), heure d'arrivée
(à minimiser) et temps de marche cumulé (à minimiser). Le nombre de correspondances est donné par le
tour k de RAPTOR (k véhicules empruntés). Au tour 0, l'heure de départ n'est pas encore fixée : elle
l'est au premier embarquement (on quitte le domicile juste à temps pour attraper le véhicule).
Le résultat est l'ensemble de Pareto des trajets respectant la fenêtre de temps.
"""
from dataclasses import dataclass, field

import numpy as np

from config_tc import MAX_ACCESS_WALK, MAX_TRAVEL, MAX_TRIPS, MIN_CHANGE, TW_WIDTH

EPS = 1e-9


class Label:
    __slots__ = ("dep", "arr", "walk", "parent", "leg")

    def __init__(self, dep, arr, walk, parent, leg):
        self.dep, self.arr, self.walk, self.parent, self.leg = dep, arr, walk, parent, leg

    def dominates(self, other):
        return self.dep >= other.dep - EPS and self.arr <= other.arr + EPS and self.walk <= other.walk + EPS


@dataclass
class Leg:
    mode: str      # "marche", "attente", "tram" ou "bus"
    line: str      # nom de la ligne (véhicule), "" sinon
    start: object  # indice d'arrêt, ou "origine" / "destination"
    end: object
    dep: float
    arr: float
    stops: list = field(default_factory=list)   # véhicule : arrêts desservis de start à end


@dataclass
class Journey:
    dep: float                # départ du point d'origine (min depuis minuit)
    arr: float                # arrivée au point de destination
    walk: float
    nb_vehicles: int
    legs: list = field(default_factory=list)   # [Leg]

    @property
    def travel(self):
        return self.arr - self.dep

    @property
    def transfers(self):
        return max(self.nb_vehicles - 1, 0)


class McRaptor:
    def __init__(self, timetable, walk_network):
        self.tt = timetable
        self.walk = walk_network

    def solve(self, access, egress, window_type, h, walk_direct):
        """access / egress : {arrêt: minutes de marche} depuis l'origine / vers la destination.
        window_type 'depart' : départ dans [h, h+15] ; 'arrivee' : arrivée dans [h-15, h]."""
        if window_type == "depart":
            dep_lo, dep_hi, arr_lo, arr_hi = h, h + TW_WIDTH, -np.inf, h + TW_WIDTH + MAX_TRAVEL
        else:
            dep_lo, dep_hi, arr_lo, arr_hi = h - TW_WIDTH - MAX_TRAVEL, h, h - TW_WIDTH, h
        self.arr_hi = arr_hi
        tt = self.tt
        bags = [dict() for _ in range(MAX_TRIPS + 1)]
        for s, a in access.items():
            bags[0][s] = [Label(None, dep_lo + a, a, None, ("marche", None, s, a))]
        marked = set(access)

        for k in range(1, MAX_TRIPS + 1):
            queue = {}
            for s in marked:
                for p_i, pos in tt.stop_patterns[s]:
                    queue[p_i] = min(queue.get(p_i, pos), pos)
            new_marked, from_vehicle = set(), {}

            for p_i, pos0 in queue.items():
                p = tt.patterns[p_i]
                route_bag = []    # [course, départ domicile, marche, étiquette parente, position d'embarquement]
                for pos in range(pos0, len(p.stops)):
                    s = p.stops[pos]
                    # Descente : les courses du sac de ligne arrivent en s.
                    for t, dep, walk, parent, b_pos in route_bag:
                        label = Label(dep, p.arr[t, pos], walk, parent, ("vehicule", p_i, t, b_pos, pos))
                        if self._insert(bags, k, s, label):
                            new_marked.add(s)
                            from_vehicle.setdefault(s, []).append(label)
                    # Montée : étiquettes du tour précédent en s.
                    col = p.dep[:, pos]
                    for prev in bags[k - 1].get(s, ()):
                        if prev.dep is None:   # premier véhicule : toutes les courses de la fenêtre
                            a = prev.walk
                            lo = np.searchsorted(col, dep_lo + a, side="left")
                            hi = np.searchsorted(col, dep_hi + a, side="right")
                            for t in range(lo, hi):
                                self._add_route(route_bag, [t, col[t] - a, a, prev, pos])
                        else:
                            t = np.searchsorted(col, prev.arr + MIN_CHANGE, side="left")
                            if t < len(col):
                                self._add_route(route_bag, [t, prev.dep, prev.walk, prev, pos])

            # Correspondances à pied après une descente.
            for s, labels in from_vehicle.items():
                for label in labels:
                    for s2, w in self.walk.footpaths[s]:
                        walked = Label(label.dep, label.arr + w, label.walk + w, label, ("marche", s, s2, w))
                        if self._insert(bags, k, s2, walked):
                            new_marked.add(s2)
            marked = new_marked
            if not marked:
                break

        # Arrivée avant la fenêtre : la personne attend, son arrivée est ramenée au début de la fenêtre
        # (l'attente compte dans le temps de trajet). Arriver plus tôt n'est donc jamais pénalisant,
        # ce qui garde valide la dominance sur l'heure d'arrivée.
        journeys = []
        for k in range(1, MAX_TRIPS + 1):
            for s, e in egress.items():
                for label in bags[k].get(s, ()):
                    reached = label.arr + e
                    arr = max(reached, arr_lo)
                    if arr <= arr_hi + EPS and arr - label.dep <= MAX_TRAVEL + EPS:
                        legs = self._legs(label) + [Leg("marche", "", s, "destination", label.arr, reached)]
                        if arr > reached + EPS:
                            legs.append(Leg("attente", "", "destination", "destination", reached, arr))
                        journeys.append(Journey(label.dep, arr, label.walk + e, k, legs))
        if walk_direct <= MAX_ACCESS_WALK:   # trajet entièrement à pied
            dep = h if window_type == "depart" else h - walk_direct
            journeys.append(Journey(dep, dep + walk_direct, walk_direct, 0,
                                    [Leg("marche", "", "origine", "destination", dep, dep + walk_direct)]))
        return pareto(journeys)

    def _insert(self, bags, k, s, label):
        if label.arr - label.dep > MAX_TRAVEL or label.arr > self.arr_hi:
            return False
        for j in range(1, k + 1):
            if any(m.dominates(label) for m in bags[j].get(s, ())):
                return False
        bag = bags[k].setdefault(s, [])
        bag[:] = [m for m in bag if not label.dominates(m)]
        bag.append(label)
        return True

    @staticmethod
    def _add_route(route_bag, entry):
        t, dep, walk = entry[:3]
        for r in route_bag:
            if r[0] <= t and r[1] >= dep - EPS and r[2] <= walk + EPS:
                return
        route_bag[:] = [r for r in route_bag if not (t <= r[0] and dep >= r[1] - EPS and walk <= r[2] + EPS)]
        route_bag.append(entry)

    def _legs(self, label):
        legs = []
        while label is not None:
            kind = label.leg[0]
            if kind == "vehicule":
                _, p_i, t, b_pos, a_pos = label.leg
                p = self.tt.patterns[p_i]
                legs.append(Leg(p.mode, p.route, p.stops[b_pos], p.stops[a_pos], p.dep[t, b_pos],
                                p.arr[t, a_pos], p.stops[b_pos:a_pos + 1]))
            else:
                _, s_from, s_to, w = label.leg
                if s_from is None:   # accès depuis l'origine : départ fixé par le premier véhicule
                    board = legs[-1].dep if legs else label.arr
                    legs.append(Leg("marche", "", "origine", s_to, board - w, board))
                else:
                    legs.append(Leg("marche", "", s_from, s_to, label.arr - w, label.arr))
            label = label.parent
        return legs[::-1]


def pareto(journeys):
    """Trajets non dominés sur (départ, arrivée, marche, nombre de véhicules)."""
    def dom(a, b):
        return (a.dep >= b.dep - EPS and a.arr <= b.arr + EPS and a.walk <= b.walk + EPS
                and a.nb_vehicles <= b.nb_vehicles)
    # Dans cet ordre, un trajet dominant (ou identique) est toujours vu avant ceux qu'il domine.
    kept = []
    for j in sorted(journeys, key=lambda j: (j.nb_vehicles, j.walk, -j.dep, j.arr)):
        if not any(dom(o, j) for o in kept):
            kept.append(j)
    return kept
