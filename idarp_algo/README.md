# idarp_algo : heuristique C++ pour le wIDARP

Deux exécutables, compilés par `make` dans `bin/` (C++17 et CPLEX 22.1.1, voir le [README racine](../README.md#installation)).
`make MODE=debug` compile en `-O0 -g` avec AddressSanitizer et UBSan, et `make MODE=profile` avec `-pg` pour gprof.

Les deux exécutables :

1. **`main_nb_vehicles`** cherche le nombre minimal de véhicules. Il part d'autant de véhicules que de
   requêtes ; chaque fois que `PartialLNS` place toutes les requêtes, il recommence avec un véhicule de moins
   que la solution trouvée, jusqu'à la limite de temps (`-t`, 300 s par défaut).
2. **`main_LNS`** optimise l'objectif avec ce nombre de véhicules plus un :
   - `PartialLNS` construit une solution initiale complète ;
   - `LNS` l'améliore en n'acceptant que des solutions complètes ;
   - les deux étapes partagent le `time_limit` de `config_instance.txt`.

## Options

Communes aux deux exécutables :

| Option | Rôle |
|---|---|
| `-f <dossier>` | dossier des fichiers d'instance (défaut `./`) |
| `-p`, `-d`, `-w` | fichiers de l'instance, des temps en voiture et des temps à pied |
| `-tram <fichier>` | horaires des trams du jour (`tram_runs_<jour>.txt`) ; remplace `-pt` |
| `-pt <fichier>` | ancien mode « matrice » : temps de tram constants entre stations, attente moyenne ajoutée |
| `-c`, `-ci` | paramètres de la recherche (`config.txt`) et de l'instance (`config_instance.txt`) |
| `-idarp` / `-darp` | modèle IDARP (sans marche) ou DARP (direct seulement) ; défaut wIDARP |
| `-ridingTime` | objectif temps de trajet des usagers ; défaut temps roulé des véhicules |
| `-s <graine>` | graine aléatoire (défaut : l'heure courante) |
| `-sol <fichier>` | écrit la solution (format ci-dessous) |

Propres à `main_nb_vehicles` : `-t <s>` pour la durée, et `-res <fichier>` pour n'écrire que le nombre de véhicules.

Propres à `main_LNS` :

| Option | Rôle |
|---|---|
| `-v <fichier>` | obligatoire : nombre minimal de véhicules, soit un fichier `-res`, soit une solution de `main_nb_vehicles` (ligne `vehicles`) |
| `-check` | vérifie la solution initiale et la meilleure avec `SolutionChecker` (sur la sortie d'erreur) |
| `-rrt <fichier>` | trace du seuil RRT : une ligne toutes les 100 itérations faisables |
| `-sp <fichier>` | trace du set partitioning (mode matrice seulement) |

`main_LNS` écrit toujours le coût final dans `objective.txt`, dans le dossier courant ; c'est ce que lit irace.
Si la solution initiale reste incomplète, il y écrit un coût volontairement très mauvais.

## Algorithme

### Recherche (`LNS`, `PartialLNS`)

- **Destruction**, tirée au hasard à chaque itération :
  - `RandomRemoval` ;
  - `WorstRemoval` (Ropke et Pisinger 2006, `worst_removal_randomness` = 3) ;
  - `RelatedRemoval`, avec deux mesures de proximité (Molenbruch et al. 2021, `related_removal_randomness` = 6).

  À chaque itération, on retire un nombre de requêtes tiré uniformément entre `nb_requests_removed_min` et
  `nb_requests_removed_max`.
- **Réparation**, tirée au hasard : `RandomRepair`, `GreedyRepair` (meilleure insertion) et `RegretRepair`
  (regret-2). `PartialLNS` n'utilise que `RandomRepair` dans `main_LNS`.
- **Acceptation (`LNS`)** : *record-to-record travel* linéaire, variante « fixed end » de Santini, Ropke et
  Hvattum (2018).
  - Une solution x' est acceptée si `(f(x') − f(best)) / f(x') < T`.
  - T décroît linéairement de `rrt_start_threshold` à `rrt_end_threshold` (0 par défaut), selon la part du
    temps ou des itérations déjà utilisée (la plus grande des deux).
- **`PartialLNS`** : acceptation gloutonne d'un score où chaque requête non placée pèse plus que tout
  l'objectif (`(2·nbVéhicules + nbRequêtes)·H + 1`).
- **Set partitioning (CPLEX, `SetPartitioning`)** : recombine les tournées rencontrées. Il ne sert qu'en mode
  matrice avec l'objectif temps roulé ; il est désactivé avec `-tram` et avec `-ridingTime`.

### Trajets d'une requête (`Instance::precomputeAccessOptions`)

Les trajets possibles avec le tram sont précalculés une fois pour toutes, avec au plus `nb_pt_options`
options par requête :

- **véhicule → tram → véhicule** : des paires de stations (`StopPair`), les deux véhicules étant reliés par un
  `Transfer` ;
- **marche → tram → véhicule** et l'inverse : les stations côté véhicule (`AccessOption`), la station de marche
  étant la plus proche (`threshold_to_walk`).

### Horaires et faisabilité (`Solution`, `DarpGraph`)

- **Calendrier** : les tournées liées par des correspondances forment un STP (problème temporel simple),
  résolu exactement par Bellman–Ford sur des contraintes de différence.
  - Avec `-tram`, chaque trajet en tram ajoute un arc dont le poids dépend de l'heure (`TramArc`) : on prend la
    première course qui part après l'heure d'arrivée à la station, ou la dernière qui arrive avant l'heure
    voulue (`TravelTimeProvider::earliestRun` / `latestRun`).
  - Ces arcs sont monotones, donc les relaxations `relaxEarliest` et `relaxLatest` restent exactes.
- **Test d'insertion en O(1)**, avant le calcul exact :
  - tournées liées par des correspondances : Masson, Lehuédé et Péton (2013), `LinkedRoutesTiming` ;
  - tournée sans correspondance : Gschwind et Drexl (2019), `RouteDarpTiming`.

  Avec les horaires, ces tests utilisent le temps de tram minimal comme borne inférieure, et le calcul exact
  tranche.
- **Temps de trajet des usagers (`-ridingTime`)** :
  - pendant la recherche, un calendrier heuristique (`ridingTimeHeuristic`) : calendrier au plus tard, prises
    en charge fixées à leur heure la plus tardive, puis calendrier au plus tôt ;
  - pour la solution finale, un LP CPLEX exact (`ridingTimeLp`) à courses de tram fixées. Les deux valeurs
    sont écrites dans le fichier solution.
- **`SolutionChecker`** : vérifie une solution à partir des seules données brutes (fenêtres, MRT, capacité,
  temps, horaires), sans réutiliser les fenêtres dérivées.

### Ordre des tests lors d'une insertion (`RequestInsertionSearch::findInsertions`)

Les opérateurs de réparation insèrent les requêtes une par une. Pour chaque requête, on cherche les meilleures
insertions faisables : 1 pour `RandomRepair` et `GreedyRepair`, 2 sur des tournées différentes pour
`RegretRepair`. Le STP exact n'attend pas que toutes les insertions soient générées. On l'appelle sur les
candidats un par un, du moins cher au plus cher, et on s'arrête dès qu'on a assez d'insertions faisables.

**Étape 1 : génération des candidats** (`InsertionEvaluator::generateCandidates`).
Pour chaque tournée et chaque couple de positions (premier arrêt, second arrêt), les tests suivants s'appliquent
dans cet ordre. On passe au couple suivant dès qu'un test échoue.

1. **Coût ajouté** (temps de conduite) : le couple est écarté si ce coût n'est pas strictement inférieur à la
   borne de coût (voir l'étape 3).
2. **Masson, Lehuédé et Péton (2013)** (`estimateLeg`, en O(1)) : vérifie les fenêtres, et les retards
   propagés aux tournées liées par des correspondances. La partie du premier arrêt ne dépend que de sa position :
   elle est calculée une seule fois pour toutes les positions du second.
3. **Gschwind et Drexl (2019)** (`admitsInRoute`, en O(1)) : la tournée vue comme une tournée du DARP, avec les
   temps de trajet maximaux de toutes ses requêtes. Ce test est exact pour une tournée sans correspondance,
   mais ce n'est qu'une condition nécessaire sinon.

Pour un trajet véhicule → tram → véhicule, chaque morceau en véhicule passe ces trois tests dans sa propre
tournée. Puis chaque combinaison de deux morceaux, sur deux tournées différentes, passe le test du
coût total, puis **l'algorithme 3 de Masson et al.** (`admitsTransfer`). Ce dernier vérifie les deux morceaux
ensemble : il détecte un cycle (proposition 4) et contrôle le temps de trajet maximal de la requête entière.

Ces tests sont des conditions nécessaires. Ils ne vérifient ni la capacité ni le calendrier complet.

**Étape 2 : vérification exacte** (`verifyInOrder` puis `verifyCandidate`).
Les candidats retenus sont examinés du moins cher au plus cher, grâce à un tas. Pour chacun :

1. on insère le candidat dans une copie de la solution ;
2. on vérifie la **capacité** des tournées modifiées ;
3. on résout le **STP** par Bellman–Ford (`propagateForwardDarp`), sur l'ensemble des tournées liées. On part des
   heures déjà calculées, ce qui accélère la résolution. Avec `-tram`, ce calcul inclut les arcs de tram.

On s'arrête dès qu'on a le nombre d'insertions faisables voulu. Les candidats plus chers ne sont jamais vérifiés.

**Étape 3 : ordre des types de trajets.**
1. On génère et on vérifie d'abord les trajets **directs** et **avec marche**, peu nombreux.
2. On génère ensuite les trajets **véhicule → tram → véhicule**, de loin les plus nombreux. On ne garde que ceux
   strictement moins chers que la pire insertion trouvée à l'étape précédente : c'est la borne de coût de
   l'étape 1. La plupart ne sont donc jamais générés.
3. On garde les meilleures insertions des deux groupes. À coût égal, un trajet direct ou avec marche passe avant
   un trajet avec deux véhicules.

**Étape 4 : application** (`apply`). L'insertion choisie est faite dans la vraie solution, avec une
correspondance (`Transfer`) si besoin. `Solution::refreshTiming` relance ensuite le STP au plus tôt et au plus
tard sur les tournées liées, et reconstruit les données des tests en O(1) pour l'insertion suivante.

Les insertions sont toujours classées par **temps de conduite ajouté**, quel que soit l'objectif. Avec
`-ridingTime`, le temps des usagers n'est calculé qu'une fois la solution réparée
(`computeObjectiveFunction`), pour décider si on l'accepte.

### Carte des fichiers

| Fichiers | Rôle |
|---|---|
| `Instance`, `Request`, `PointRegistry` | lecture de l'instance, requêtes, numérotation des points, trajets précalculés |
| `TravelTimeProvider` | seul accès aux temps (voiture, marche, tram) et aux horaires des courses |
| `Route`, `Transfer`, `Solution` | tournées, correspondances, calendrier (STP), objectifs, écriture du fichier solution |
| `LinkedRoutesTiming`, `RouteDarpTiming`, `InsertionEvaluator`, `RequestInsertionSearch` | tests d'insertion et recherche de la meilleure insertion |
| `Destroyer` et `*Removal` ; `Repairer` et `*Repair` | opérateurs de destruction et de réparation |
| `LNS`, `PartialLNS`, `ConfLNS` | boucles de recherche et lecture des paramètres |
| `RoutePool`, `SetPartitioning` | set partitioning (mode matrice) |
| `SolutionChecker` | vérification indépendante |
| `main_nb_vehicles.cpp`, `main_LNS.cpp` | exécutables |

## Fichiers

### Entrées (produits par `creation_instance_le_havre/03_generation.py`)

- `i<n>_<n>_<k>.txt` :
  - en-tête `nbRequêtes nbVéhicules nbStations capacité horizon` ;
  - puis une ligne par requête : `idP ltwP utwP idD ltwD utwD MRT quantité tempsService`.

  Les temps sont en minutes depuis 6h00, avec un horizon de 840.
- `d…`, `w…` : matrices des temps en voiture et à pied. Les points sont numérotés ainsi : dépôt 0, pickup de
  la requête r en 2r+1, delivery en 2r+2, station s en 1+2n+s.
- `tram_runs_<jour>.txt` : nombre de courses, puis une course par ligne :
  `ligne nbArrêts station heure station heure …`.
- `public_transport_time_<jour>.txt` : matrice des temps de tram, pour l'ancien mode `-pt` seulement.

### Paramètres

`config.txt` (option `-c`), réglé par irace :

| Clé | Sens |
|---|---|
| `nb_requests_removed_min`, `nb_requests_removed_max` | nombre de requêtes retirées par itération (borné par la taille de l'instance) |
| `rrt_start_threshold` | seuil RRT de départ (écart relatif à la meilleure solution) |
| `rrt_end_threshold` | facultatif, 0 par défaut |
| `nb_pt_options` | nombre maximal de trajets en tram gardés par requête |
| `worst_removal_randomness`, `related_removal_randomness`, `set_partitioning_pool_ratio` | facultatifs |

`config_instance.txt` (option `-ci`) :

| Clé | Sens |
|---|---|
| `time_limit` | durée de `main_LNS`, en secondes |
| `iteration_limit` | limite d'itérations ; 0 ou absente pour ne pas en avoir |
| `threshold_to_walk` | marche maximale entre un point et la station la plus proche (min) |
| `minimum_time_in_transport_public` | temps de tram minimal pour qu'un trajet en tram soit proposé (min) |
| `waiting_time` | attente moyenne ajoutée en mode matrice ; ignorée avec `-tram` |

### Solution (`-sol`, écrite par `Solution::writeFile`)

```
instance i30_30_0.txt          ← en-tête : instance, pt (tram ou matrix), model, seed, objective, [vehicles],
...                              objective_type, route_time, riding_time, riding_time_lp
routes <nbRoutes>
route <r> <nbArrêts>           ← puis un arrêt par ligne : <P|D|SB|SA> <requête> <point> <heure>
requests <nbRequêtes>          ← puis une ligne par requête :
<id> direct <route>
<id> walk_pickup <route> <stationProche> <marche> <départ> <départTram> <arrivéeTram> <station>
<id> walk_delivery <route> <station> <départTram> <arrivéeTram> <stationProche> <marche> <arrivée>
<id> transfer <routeAvant> <routeAprès> <stationAvant> <stationAprès> <départTram> <arrivéeTram>
<id> unassigned
```

Les codes d'arrêt : `P` prise en charge, `D` dépôt, `SB` descente du véhicule pour le tram, `SA` montée dans le
véhicule après le tram. Le passage au dépôt est implicite.

`../creation_instance_le_havre/solution_idarp.py <solution> <dossier d'instances>` relit ce fichier et le
vérifie indépendamment du C++ :
- temps de conduite, capacité, fenêtres, MRT et marche ;
- existence des courses de tram ;
- recalcul de `route_time` et de `riding_time`.

## Expériences

### `run_tests.sh` : tous les jours et toutes les instances, en local

```bash
./run_tests.sh 1        # main_nb_vehicles -> ../results/lehavre_sol_init/<modèle>/<jour>/sol_init_<inst>.txt
./run_tests.sh 2        # main_LNS         -> ../results/lehavre_rural/<modèle>/<objectif>/<jour>/sol_<inst>.txt
./run_tests.sh          # les deux phases
```

Les résultats déjà présents sont sautés. Tous les réglages se passent par variables d'environnement :

| Variable | Valeurs |
|---|---|
| `NB_JOBS` | processus en parallèle (3 par défaut) |
| `MODELE` | `widarp` (défaut), `idarp`, `darp` |
| `OBJECTIF` | `route_time` (défaut), `riding_time` |
| `JEU` | `test` (défaut, `lehavre_rural`) ou `irace` (`lehavre_rural_irace`, résultats suffixés `_irace`) |
| `DAYS`, `INSTANCES` | jours et instances à traiter ; par défaut tous, toutes tailles confondues |
| `SEED`, `NB_VEHICLES_TIME`, `CONFIG`, `CONFIG_INSTANCE` | graine (0), durée de la phase 1 (300 s), fichiers de paramètres |

Par exemple :

```bash
INSTANCES="$(seq -f '60_60_%g' 0 9)" CONFIG=<config réglée>.txt NB_JOBS=6 ./run_tests.sh 2
```

### Réglage avec irace (`iraceFolder/`)

| Fichier | Rôle |
|---|---|
| `scenario.txt` | budget de 3000 expériences, 20 en parallèle ; `parameterFile` et `trainInstancesFile` doivent avoir la même taille n |
| `parameters_<n>.txt` | paramètres réglés : `nb_requests_removed_min` (0 à n−5), `largeur_requests_removed` (0 à 20), `rrt_start_threshold` (0,001 à 0,2, échelle log), `nb_pt_options` (1 à 40) |
| `instances_<n>.txt` | les 36 couples `<jour>/i<n>_<n>_<k>.txt` du jeu d'apprentissage |
| `config_instance.txt` | 10 min par évaluation, sans limite d'itérations |
| `target-runner` | écrit `config.txt` dans `run_<config>_<instance>/`, lance `main_LNS` en mode `-tram` avec le nombre de véhicules de `results/lehavre_sol_init_irace/`, et affiche `objective.txt` |

Avant de lancer irace, il faut le nombre de véhicules du jeu d'apprentissage :
`JEU=irace INSTANCES="…" ./run_tests.sh 1`.

Variables de `target-runner` : `IRACE_MODELE`, `IRACE_OBJECTIF` et `IRACE_CONFIG_INSTANCE`.

### Serveur SLURM (`run_serveur.sh`)

```bash
make                       # après chaque mise à jour du code, pas pendant un run
sbatch run_serveur.sh      # depuis idarp_algo/ : 20 cœurs, 8 Go, 3 jours au plus
```

Il lance irace avec `--parallel` égal au nombre de cœurs demandés, et le journal va dans `irace_<job>.out`.
La meilleure configuration figure à la fin de ce journal. Pour en faire un `config.txt` :
`nb_requests_removed_max = nb_requests_removed_min + largeur_requests_removed`.
