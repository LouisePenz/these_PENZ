"""Télécharge les données brutes dans data/raw/ (les fichiers déjà présents sont conservés)."""
import json
import zipfile

import py7zr
import requests

from config import EPCI_CODE, RAW_DIR

SOURCES = {
    # Réseau routier et piéton (OpenStreetMap, extrait Geofabrik)
    "haute-normandie.osm.pbf": "https://download.geofabrik.de/europe/france/haute-normandie-latest.osm.pbf",
    # Population carroyée à 200 m (INSEE, Filosofi 2019)
    "filosofi_200m.zip": "https://www.insee.fr/fr/statistiques/fichier/7655475/Filosofi2019_carreaux_200m_gpkg.zip",
    # Flux domicile-travail entre communes (INSEE, MOBPRO 2023)
    "mobpro.zip": "https://www.insee.fr/fr/statistiques/fichier/8998300/base-flux-mobilite-domicile-lieu-travail-2023_csv.zip",
}
COMMUNES_URL = (f"https://geo.api.gouv.fr/epcis/{EPCI_CODE}/communes"
                "?fields=nom,code,population,centre&format=geojson&geometry=contour")


def download(url, dest):
    if dest.exists():
        print(f"[déjà là] {dest.name}")
        return
    print(f"[téléchargement] {dest.name} <- {url}")
    tmp = dest.with_suffix(dest.suffix + ".part")
    with requests.get(url, stream=True, timeout=60) as r:
        r.raise_for_status()
        with open(tmp, "wb") as f:
            for chunk in r.iter_content(chunk_size=1 << 20):
                f.write(chunk)
    tmp.rename(dest)


def main():
    RAW_DIR.mkdir(parents=True, exist_ok=True)
    for name, url in SOURCES.items():
        download(url, RAW_DIR / name)
        if name.endswith(".zip"):
            out = RAW_DIR / name[:-4]
            if not out.exists():
                with zipfile.ZipFile(RAW_DIR / name) as z:
                    z.extractall(out)
            # Filosofi : le zip contient lui-même une archive 7z
            for archive in out.glob("*.7z"):
                with py7zr.SevenZipFile(archive) as z:
                    z.extractall(out)
                archive.unlink()

    communes = RAW_DIR / "communes_cu.geojson"
    if not communes.exists():
        r = requests.get(COMMUNES_URL, timeout=60)
        r.raise_for_status()
        communes.write_text(json.dumps(r.json()))
    print("Terminé.")


if __name__ == "__main__":
    main()
