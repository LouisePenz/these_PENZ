"""Temps de parcours en voiture plus proches du réel que les temps à vitesse limite.

Sur chaque tronçon : longueur / (vitesse limite OSM x coefficient du type de route), plus une pénalité
fixe par intersection (sauf voies rapides), qui représente feux, priorités et giratoires.
"""
from config import DRIVE_INTERSECTION_PENALTY, DRIVE_SPEED_FACTORS


def set_realistic_travel_times(drive):
    """Remplace l'attribut travel_time (s) des arêtes du graphe routier osmnx."""
    for _, _, data in drive.edges(data=True):
        highway = data["highway"][0] if isinstance(data["highway"], list) else data["highway"]
        speed_ms = float(data["speed_kph"]) * DRIVE_SPEED_FACTORS.get(highway, 0.7) / 3.6
        penalty = 0 if highway.startswith(("motorway", "trunk")) else DRIVE_INTERSECTION_PENALTY
        data["travel_time"] = float(data["length"]) / speed_ms + penalty
    return drive
