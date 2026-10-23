"""Paramètres communs à la préparation des données et à la génération des instances."""
from pathlib import Path

ROOT = Path(__file__).resolve().parent
GTFS_DIR = ROOT / "GTFS"
RAW_DIR = ROOT / "data" / "raw"          # fichiers téléchargés (non versionnés)
PREP_DIR = ROOT / "data" / "prepared"    # fichiers dérivés (non versionnés)
INSTANCES_DIR = ROOT.parent / "instances" / "lehavre_rural"

# --- Territoire --------------------------------------------------------------
EPCI_CODE = "200084952"        # Communauté urbaine Le Havre Seine Métropole
CODE_LE_HAVRE = "76351"        # commune de travail pour les flux MOBPRO
HOTEL_DE_VILLE = (0.107290, 49.493950)   # (lon, lat)
MAX_DRIVE_FROM_CENTER = 30     # min, borne dure sur la commune d'origine

# --- Marche / voiture --------------------------------------------------------
WALK_SPEED_KMH = 4.5
NEAR_TRAM_MAX_WALK = 5         # min, point "ville" : proche d'un arrêt de tram
FAR_TRAM_MIN_WALK = 15         # min, point "rural" : loin de tout arrêt de tram

# Voiture : vitesse effective = vitesse limite OSM x coefficient du type de route,
# plus une pénalité par intersection (hors voies rapides). Voir temps_voiture.py.
DRIVE_SPEED_FACTORS = {
    "motorway": 0.9, "motorway_link": 0.8, "trunk": 0.85, "trunk_link": 0.8,
    "primary": 0.75, "primary_link": 0.7, "secondary": 0.75, "secondary_link": 0.7,
    "tertiary": 0.7, "tertiary_link": 0.7, "unclassified": 0.8,
    "residential": 0.6, "living_street": 0.5, "road": 0.6,
}
DRIVE_INTERSECTION_PENALTY = 2  # s

# --- Jours de référence (GTFS LiA) ------------------------------------------
DAYS = {
    "lundi_hors_scolaire": "2026-10-26",
    "lundi_scolaire": "2026-11-16",
    "samedi_scolaire": "2026-11-14",
}

# --- Requêtes ---------------------------------------------------------------
# Les temps des instances sont en minutes depuis TIME_ORIGIN.
TIME_ORIGIN = 6 * 60           # 6h00
HORIZON = 14 * 60              # 6h00 -> 20h00
PERIODS = [                    # (début, fin, début du pic, fin du pic), en minutes depuis minuit
    (7 * 60, 11 * 60, 8 * 60, 8 * 60 + 30),
    (15 * 60, 19 * 60, 17 * 60, 18 * 60),
]
PEAK_SHARE = 0.25              # part des requêtes d'une période tombant dans son pic
TW_WIDTH = 15                  # min
MIN_DIRECT_DRIVE = 10          # min, trajet direct en voiture minimum (requête retirée sinon)
MRT_RATIO = 2                  # MRT = ratio * temps direct en voiture + temps de service
SERVICE_TIME = 1               # min
QUANTITY_PROBS = {1: 0.7, 2: 0.2, 3: 0.1}

# --- Flotte (mode 2, format wIDARP) -----------------------------------------
DEPOT = (0.1058796, 49.5299138)  # (lon, lat), dépôt des anciennes instances
CAPACITY = 6
