# Création des instances Le Havre (périurbain / rural ↔ tram)

Paramètres : `config.py`. Environnement : `.venv/` (pandas, geopandas, osmnx, osmium, r5py, py7zr, scipy).

```bash
.venv/bin/python 01_telechargement.py   # données brutes -> data/raw/
.venv/bin/python 02_preparation.py      # réseaux, zones, pondérations -> data/prepared/
.venv/bin/python 03_generation.py -n 30 -k 10 -s 0   # -> ../instances/lehavre_rural/
```

## Données

| Donnée | Source |
|---|---|
| Réseau TC (horaires) | GTFS LiA, dossier `GTFS/` (22/10/2026 → 03/01/2027) |
| Réseaux piéton et routier | OpenStreetMap, extrait Geofabrik Haute-Normandie |
| Population | INSEE, Filosofi 2019, carreaux de 200 m |
| Navetteurs vers Le Havre | INSEE, MOBPRO 2023 (flux domicile-travail) |
| Communes de la CU | geo.api.gouv.fr, EPCI 200084952 |

## Temps en voiture

Les temps en voiture ne sont pas pris à la vitesse limite. La vitesse effective est la vitesse limite OSM
multipliée par un coefficient qui dépend du type de route : 0,6 en zone résidentielle, 0,7 à 0,75 sur les
routes principales, 0,9 sur autoroute. On ajoute 2 s par intersection, sauf sur les voies rapides
([temps_voiture.py](temps_voiture.py), paramètres dans `config.py`). Temps obtenus depuis l'Hôtel de Ville :

| Destination | Gare | Harfleur | Octeville | Gonfreville | Montivilliers | Saint-Romain |
|---|---|---|---|---|---|---|
| Temps | 4,4 min | 15 min | 16 min | 18,5 min | 21 min | 29 min |

## Requêtes

- Un point « ville » à ≤ 5 min à pied d'une station de tram : station tirée uniformément, puis un nœud piéton proche.
- Un point « rural » à > 15 min à pied de toute station et à ≤ 30 min en voiture de l'Hôtel de Ville.
  Carreau tiré avec un poids `flux MOBPRO(commune → Le Havre) × pop. du carreau / pop. de la commune`,
  puis un nœud piéton du carreau.
- Trajet direct en voiture ≥ 10 min, sinon la requête est rejetée et tirée à nouveau.
- Sens au hasard : entrant (rural → ville) ou sortant (ville → rural).
- Heure : 7h–11h ou 15h–19h (50/50), dont 25 % dans le pic (8h–8h30, 17h–18h).
- Fenêtre de 15 min au départ **ou** à l'arrivée (50/50). L'autre fenêtre est déduite pour le format wIDARP
  (temps direct en voiture T, MRT = 2 T + 1).
- Nombre de personnes : 1, 2 ou 3 (70/20/10 %).

## Jours de référence

Les mêmes requêtes servent pour les trois jours ; seuls `public_transport_time_<jour>.txt` et `tram_runs_<jour>.txt` changent.

| Jour | Date | Services GTFS |
|---|---|---|
| `lundi_hors_scolaire` | 26/10/2026 | `13_0` |
| `lundi_scolaire` | 16/11/2026 | `12_0`, `12_9` |
| `samedi_scolaire` | 14/11/2026 | `12_6` |

## Fichiers produits (format `idarp_algo`, temps en minutes depuis 6h00, horizon 840)

- `i{n}_{n}_{k}.txt` : en-tête `nbRequêtes nbVéhicules nbStations capacité horizon`, puis par requête
  `idP ltwP utwP idD ltwD utwD MRT quantité tempsService`.
- `d…` / `w…` : temps en voiture / à pied entre dépôt (0), pickups (2r+1), deliveries (2r+2) et stations (1+2n+s).
- `public_transport_time_<jour>.txt` : temps en tram entre stations, meilleur temps en véhicule sur les courses du jour
  (mode statique de `idarp_algo`, option `-pt`, avec `waiting_time`).
- `tram_runs_<jour>.txt` : toutes les courses de tram du jour avec leurs heures de passage (option `-tram` de
  `idarp_algo`). Première ligne : nombre de courses ; puis une course par ligne,
  `ligne nbArrêts station heure station heure …` (stations dans l'ordre de `tram_stations.geojson`).
- `l….geojson` : coordonnées des points ; `r….csv` : fenêtre d'origine, sens, commune, etc. (pour le mode TC).

Exemple, en mode horaires (un seul tram par usager, sans set partitioning) :

```bash
cd ../idarp_algo
./bin/main_LNS -f ../instances/lehavre_rural/ -p i30_30_0.txt -d d30_30_0.txt -w w30_30_0.txt \
    -tram tram_runs_lundi_scolaire.txt -c config.txt -ci config_instance.txt -v <fichier nb véhicules> -check
```
