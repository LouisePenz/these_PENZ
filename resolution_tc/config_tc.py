"""Paramètres de la résolution transports en commun + marche (mode 1)."""
from pathlib import Path

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parent
GTFS_DIR = REPO / "creation_instance_le_havre" / "GTFS"
WALK_GRAPH = REPO / "creation_instance_le_havre" / "data" / "prepared" / "walk.graphml"
INSTANCES_DIR = REPO / "instances" / "lehavre_rural"
RESULTS_DIR = REPO / "results" / "lehavre_rural_tc"
CACHE_DIR = ROOT / "cache"

# Type de jour -> date de référence du GTFS LiA
DAY_TYPES = {
    "vacances": "2026-10-26",   # lundi hors période scolaire
    "scolaire": "2026-11-16",   # lundi en période scolaire
    "samedi": "2026-11-14",     # samedi en période scolaire
}

TIME_ORIGIN = 6 * 60          # les instances comptent les minutes depuis 6h00
TW_WIDTH = 15                 # min

WALK_SPEED_KMH = 4.5
MAX_ACCESS_WALK = 30          # min, marche maximale entre le domicile (ou la destination) et un arrêt
MAX_TRANSFER_WALK = 10        # min, marche maximale entre deux arrêts lors d'une correspondance
MAX_STOP_SNAP = 200           # m, au-delà un arrêt GTFS n'est pas relié au réseau piéton
MIN_CHANGE = 1                # min, temps minimal de correspondance
MAX_TRIPS = 5                 # nombre maximal de véhicules (donc MAX_TRIPS - 1 correspondances)
MAX_TRAVEL = 180              # min, durée maximale d'un trajet porte à porte
