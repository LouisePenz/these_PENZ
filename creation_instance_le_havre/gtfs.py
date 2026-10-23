"""Lecture du GTFS LiA complété par la ligne de tram C (GTFS_ligne_C, voir 00_ligne_c.py)."""
import pandas as pd

from config import GTFS_DIR, GTFS_LIGNE_C_DIR


def read(name, **kwargs):
    """Table GTFS `name` (ex. "stops.txt") : celle de LiA, suivie de celle de la ligne C si elle existe."""
    frames = [pd.read_csv(d / name, **kwargs) for d in (GTFS_DIR, GTFS_LIGNE_C_DIR) if (d / name).exists()]
    return pd.concat(frames, ignore_index=True)
