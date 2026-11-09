# wIDARP au Havre : instances, résolution et réglage

Code de la thèse sur le transport à la demande intégré aux transports en commun (wIDARP) :
véhicules flexibles, tram et marche. Les instances sont construites sur le Havre, avec des requêtes
entre le périurbain ou le rural et le centre-ville.

## Chaîne de traitement

```
creation_instance_le_havre/   Python : données ouvertes -> instances (requêtes, temps voiture/marche, horaires des trams)
        │
        ├──> resolution_tc/   Python : base de comparaison, transports en commun + marche seuls (McRAPTOR)
        │
        └──> idarp_algo/      C++ : 1. main_nb_vehicles, nombre minimal de véhicules
                                    2. main_LNS, LNS sur ce nombre de véhicules
                              + réglage des paramètres avec irace, lancement sur le serveur SLURM
                 │
                 v
             results/  ──>  creation_instance_le_havre/05_visualisation.py (carte interactive)
                            creation_instance_le_havre/solution_idarp.py   (vérification indépendante)
```

| Dossier | Contenu | Détail |
|---|---|---|
| [creation_instance_le_havre/](creation_instance_le_havre/) | Téléchargement, préparation des réseaux, génération des instances, carte, vérificateur de solutions | [README](creation_instance_le_havre/README.md) |
| [resolution_tc/](resolution_tc/) | Meilleurs trajets en transports en commun et à pied (horaires GTFS réels), statistiques | [README](resolution_tc/README.md) |
| [idarp_algo/](idarp_algo/) | Heuristique C++ : minimisation de la flotte, LNS, horaires des trams, scripts d'expériences et irace | [README](idarp_algo/README.md) |
| `instances/` | Instances générées (non versionnées) | ci-dessous |
| `results/` | Solutions et résultats (non versionnés) | ci-dessous |

## Modèle en bref

- **Requêtes** : une extrémité à 5 min ou moins à pied d'une station de tram (ville), l'autre à plus de
  15 min (périurbain ou rural), dans les deux sens. Une seule fenêtre de 15 min, au départ ou à l'arrivée.
  Temps de trajet maximal MRT = 2 T + 1, où T est le temps direct en voiture. De 1 à 3 personnes.
- **Trajets possibles** (option du modèle) :
  - `widarp` (défaut) : direct, véhicule → tram → véhicule, marche → tram → véhicule et l'inverse ;
  - `idarp` (`-idarp`) : direct ou véhicule → tram → véhicule, sans marche ;
  - `darp` (`-darp`) : direct seulement.
- **Tram** : horaires réels du jour (lignes A et B, plus la ligne C en projet), un seul tram par usager.
  Chaque passage choisi doit exister dans les horaires.
- **Objectifs** :
  - `route_time` (défaut) : temps roulé des véhicules, sans les attentes ;
  - `riding_time` (`-ridingTime`) : temps de trajet des usagers, de leur prise en charge par un véhicule
    jusqu'à leur dépôt ou leur arrivée à pied.
- **Trois jours de référence**, avec les mêmes requêtes : `lundi_scolaire` (16/11/2026),
  `lundi_hors_scolaire` (26/10/2026), `samedi_scolaire` (14/11/2026). Seuls les horaires du tram changent.

## Installation

| Outil | Pour | Remarque |
|---|---|---|
| Python 3.12 et un venv dans `creation_instance_le_havre/.venv` | instances, TC, carte, vérification | pandas, geopandas, osmnx, osmium, scipy, py7zr, r5py |
| g++ (C++17) et IBM CPLEX Studio 22.1.1 | `idarp_algo` | le `Makefile` cherche `/opt/ibm/ILOG/CPLEX_Studio2211`, puis `~/CPLEX_Studio2211` ; sinon `make CPLEXDIR=…` |
| R avec irace 4.4.3 | réglage des paramètres | `install.packages("irace")` |

```bash
cd creation_instance_le_havre && python3 -m venv .venv && .venv/bin/pip install pandas geopandas osmnx osmium scipy py7zr r5py
cd ../idarp_algo && make          # -> bin/main_nb_vehicles, bin/main_LNS
```

## Démarrage rapide : une instance de bout en bout

À lancer depuis `idarp_algo/`, une fois les instances présentes dans `../instances/lehavre_rural/` :

```bash
# 1. nombre minimal de véhicules (5 min), et la solution correspondante
./bin/main_nb_vehicles -f ../instances/lehavre_rural/ -p i30_30_0.txt -d d30_30_0.txt -w w30_30_0.txt \
    -tram tram_runs_lundi_scolaire.txt -c config.txt -ci config_instance.txt -s 0 -sol sol_init.txt
# 2. LNS avec ce nombre de véhicules (lu sur la ligne "vehicles" de sol_init.txt)
./bin/main_LNS -f ../instances/lehavre_rural/ -p i30_30_0.txt -d d30_30_0.txt -w w30_30_0.txt \
    -tram tram_runs_lundi_scolaire.txt -c config.txt -ci config_instance.txt -v sol_init.txt -s 0 -sol sol.txt
# 3. vérification indépendante, puis carte
cd ../creation_instance_le_havre
.venv/bin/python solution_idarp.py ../idarp_algo/sol.txt ../instances/lehavre_rural
```

Pour toutes les instances et tous les jours, utiliser `idarp_algo/run_tests.sh` (voir le
[README d'idarp_algo](idarp_algo/README.md#expériences)).

## Visualiser les instances et les solutions

La carte affiche les instances de `instances/lehavre_rural/` (le jeu de test) sur un fond OpenStreetMap,
avec les solutions disponibles dans `results/`. Elle s'ouvre dans le navigateur.

1. **Installer l'environnement Python** une fois (voir [Installation](#installation)).
2. **Mettre les solutions à afficher dans `results/`** : la carte trouve d'elle-même tout ce qui s'y trouve, avec
   les noms de fichiers de `run_tests.sh` et de `resoudre_tc.py`.
   - `results/lehavre_rural_tc/<scolaire|vacances|samedi>/solutions_<instance>.json` : trajets en transports en
     commun seuls ;
   - `results/lehavre_sol_init/<modèle>/<jour>/sol_init_<instance>.txt` : solutions de `main_nb_vehicles` ;
   - `results/lehavre_rural/<modèle>/<objectif>/<jour>/sol_<instance>.txt` : solutions de `main_LNS`. Les runs
     faits sur le serveur doivent être recopiés dans ce dossier.
3. **Lancer la carte**, depuis `creation_instance_le_havre/` :
   ```bash
   .venv/bin/python 05_visualisation.py              # ouvre http://localhost:8765/visualisation.html
   .venv/bin/python 05_visualisation.py -i 30_30_0   # avec cette instance sélectionnée à l'ouverture
   ```
   Le script écrit `instances/lehavre_rural/visualisation.html`, puis le sert en local et l'ouvre dans le
   navigateur. Il faut une connexion internet pour le fond de carte et la bibliothèque Leaflet. **Ctrl+C**
   dans le terminal arrête le serveur. Si une nouvelle solution arrive dans `results/`, relancer le script.
4. **Utiliser la carte** :
   - **Instance** : choisir l'instance. Les requêtes s'affichent en traits directs du pickup à la delivery, et
     le tableau à gauche donne leur détail.
   - **Solution** : choisir une solution parmi les trajets TC (par jour, avec le critère temps, marche ou
     correspondances), les solutions initiales et les solutions LNS (par modèle, objectif et jour). Chaque
     requête est alors tracée selon son trajet : véhicule, tram ou bus, marche, avec un symbole à chaque
     montée ou descente.
   - **Cliquer sur une requête** (sur la carte ou dans le tableau) l'isole : les autres passent en transparence
     et son détail s'affiche.
   - **« tournées des véhicules (wIDARP) »** affiche les tournées, avec les arrêts numérotés. Cliquer sur une
     tournée l'isole.
   - **Filtres** : sens (entrant ou sortant), période (matin ou soir), type de fenêtre, et affichage des
     lignes de tram, des stations et du dépôt.
   - **L'adresse de la page** garde la sélection et peut être partagée, par exemple :
     `http://localhost:8765/visualisation.html#30_30_0/tc:scolaire/temps/12` (requête 12), ou
     `…#30_30_0/route_time-widarp:lundi_scolaire/temps/v2` (tournée du véhicule 2).

Pour vérifier une solution sans carte :
`.venv/bin/python solution_idarp.py <fichier sol> ../instances/lehavre_rural` affiche les violations
éventuelles, ou « Aucune violation. ».

## Données

Le dépôt contient, en plus du code, les horaires de tram et de bus, les instances et les résultats. On peut
donc lancer les expériences et afficher les solutions sans rien télécharger.

De `creation_instance_le_havre/data/` (1,6 Go de données OSM et INSEE brutes, et de réseaux préparés), le dépôt
ne garde que quatre fichiers, utilisés par la carte et par `resolution_tc` :
- `data/prepared/walk.graphml` (réseau piéton, 41 Mo) ;
- `data/prepared/tram_stations.geojson` ;
- `data/prepared/tram_lignes.json` ;
- `data/raw/communes_cu.geojson`.

Le reste ne sert qu'à regénérer les instances. On le recrée avec `01_telechargement.py`, puis
`02_preparation.py`. La génération des instances est déterministe (graines fixes) et redonne les mêmes fichiers.

| Dossier | Contenu | Produit par |
|---|---|---|
| `creation_instance_le_havre/GTFS/` | horaires LiA (bus et trams A et B), du 22/10/2026 au 03/01/2027 | téléchargé |
| `creation_instance_le_havre/GTFS_ligne_C/` | ligne de tram C, en projet | `00_ligne_c.py` |
| `instances/` | instances (détail ci-dessous) | `03_generation.py` |
| `results/` | solutions et statistiques (détail ci-dessous) | `run_tests.sh`, `resolution_tc/resoudre_tc.py` |

Les jeux d'instances :

- `instances/lehavre_rural/` : jeu de test, avec 10 instances de 30 requêtes et 10 de 60
  (`03_generation.py -n <n> -k 10 -s 0`), plus les fichiers `tram_runs_<jour>.txt`,
  `public_transport_time_<jour>.txt` et `instances.csv` (période matin ou soir de chaque instance) ;
- `instances/lehavre_rural_irace/` : jeu d'apprentissage d'irace, distinct du jeu de test, avec 12 instances
  de 30, de 60 et de 100 requêtes, équilibrées entre le matin et le soir
  (`03_generation.py -n <n> -k 12 -s 1 --equilibrer -o ../instances/lehavre_rural_irace`) ;
- `instances/lehavre/` : anciennes instances, en mode matrice de temps de tram, conservées pour mémoire.

Les résultats :

- `results/lehavre_sol_init[_irace]/<modèle>/<jour>/sol_init_<instance>.txt` : solutions de `main_nb_vehicles`
  (nombre minimal de véhicules) ;
- `results/lehavre_rural[_irace]/<modèle>/<objectif>/<jour>/sol_<instance>.txt` : solutions de `main_LNS` ;
- `results/lehavre_rural_tc/<type de jour>/` : trajets et statistiques en transports en commun seuls.
