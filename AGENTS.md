# Consignes pour les assistants de code (IA)

Lire d'abord [README.md](README.md), puis le README du dossier concerné
([idarp_algo](idarp_algo/README.md), [creation_instance_le_havre](creation_instance_le_havre/README.md),
[resolution_tc](resolution_tc/README.md)).

## Vérifier un changement

```bash
cd idarp_algo && make                     # -Wall -Werror : aucun avertissement toléré
# run court avec vérification interne (mettre time_limit:60 dans une copie de config_instance.txt)
./bin/main_LNS -f ../instances/lehavre_rural/ -p i30_30_0.txt -d d30_30_0.txt -w w30_30_0.txt \
    -tram tram_runs_lundi_scolaire.txt -c config.txt -ci <config_instance 60 s> \
    -v ../results/lehavre_sol_init/widarp/lundi_scolaire/sol_init_30_30_0.txt -s 0 -check -sol /tmp/sol.txt
# vérification indépendante (Python, à partir des seuls fichiers de l'instance)
cd ../creation_instance_le_havre && .venv/bin/python solution_idarp.py /tmp/sol.txt ../instances/lehavre_rural
```

Il n'y a pas de tests unitaires. `SolutionChecker` (avec `-check`) et `solution_idarp.py` servent de tests :
une solution doit passer les deux sans violation. `-check` doit afficher `0 violation(s)` pour les calendriers
au plus tôt et au plus tard, et `solution_idarp.py` doit afficher « Aucune violation. ».

## Invariants à respecter

- **Temps** : entiers, en minutes depuis 6h00, avec un horizon de 840.
- **Numérotation des points** dans les matrices `d`/`w` :
  - dépôt 0 ;
  - pickup de la requête r en 2r+1, delivery en 2r+2 ;
  - station s en 1+2n+s.

  Les stations ont aussi un indice local 0..nbStations−1 (`PointRegistry`), utilisé par les horaires et la
  matrice de tram.
- **Accès aux données** : passer uniquement par `TravelTimeProvider` pour les temps et les horaires. Avec
  `-tram`, `kNoTram` signale l'absence de course.
- **Tram** : un seul tram par usager, sans correspondance A ↔ B. Avec `-tram`, `waiting_time` est forcé à 0, et
  chaque course utilisée doit exister dans `tram_runs_<jour>.txt`.
- **Après toute modification des tournées**, appeler `Solution::refreshTiming`, sinon les données des tests
  O(1) sont fausses.
- **Set partitioning** : seulement en mode matrice (`-pt`) avec l'objectif `route_time`. Il est incompatible
  avec les horaires et avec `-ridingTime`.
- **Format de fichier solution** : il est lu par `solution_idarp.py` et `05_visualisation.py`. Toute modification
  doit être reportée dans ces deux scripts et dans le commentaire de `Solution::write`.
- **Clés de configuration** : elles sont lues par `ConfLNS` avec `.at()`, donc une clé obligatoire absente
  arrête le programme. Une clé ajoutée ou renommée doit être reportée dans `config.txt`, dans
  `iraceFolder/target-runner` et dans `iraceFolder/parameters_<n>.txt`.

## Pièges connus

- **Runs en cours** : ne jamais modifier `bin/`, `run_tests.sh` ni `target-runner` pendant qu'un run tourne.
  Bash relit les scripts en cours d'exécution, et la binaire serait remplacée sous le run. Compiler ailleurs,
  ou attendre la fin.
- **Chemins** : `target-runner` et `run_tests.sh` travaillent en chemins absolus, parce que `main_LNS` tourne
  dans un dossier de travail à part, où il écrit `objective.txt`.
- **Tailles d'instances** : `run_tests.sh` traite par défaut toutes les tailles présentes dans le dossier
  d'instances. Restreindre avec `INSTANCES`.
- **Dépôt git** : `instances/` et `results/` sont versionnés, et servent de référence. Ne pas les écraser sans
  le demander. Seul `creation_instance_le_havre/data/` (1,6 Go, régénérable) est hors du dépôt.

## Conventions

- C++ :
  - C++17, avec les commentaires en anglais ;
  - un en-tête par classe dans `include/` et son implémentation dans `src/` ;
  - commentaires de conception au-dessus des classes et des fonctions, avec les références des articles.
- Scripts Python et shell, README et messages de commit : en français.
- Garder le style et la densité de commentaires du code voisin.
