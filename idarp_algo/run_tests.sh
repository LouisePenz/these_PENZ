#!/bin/bash
# Instances Le Havre périurbain / rural (../instances/lehavre_rural), mode horaires des trams (-tram),
# pour chaque jour de référence :
#   1. main_nb_vehicles : nombre minimal de véhicules, et la solution correspondante
#      -> ../results/lehavre_rural_idarp/<jour>/nb_vehicules/i<instance>.txt
#      -> ../results/lehavre_sol_init/<jour>/sol_init_<instance>.txt
#   2. main_LNS avec ce nombre de véhicules (+1, voir -v) : la meilleure solution
#      -> ../results/lehavre_rural_idarp/<jour>/sol_<instance>.txt
# Les résultats déjà présents sont sautés. Usage : ./run_tests.sh [phase : 1, 2 ou tout (défaut)]
# Chaque réglage ci-dessous peut être remplacé par une variable d'environnement, par exemple :
#   DAYS=lundi_scolaire INSTANCES="30_30_0 30_30_1" NB_JOBS=8 ./run_tests.sh 1

# Nombre de processus en parallèle
NB_JOBS=${NB_JOBS:-3}
# Graine (reproductibilité) et durée de main_nb_vehicles (s) ; durée de main_LNS : config_instance.txt
SEED=${SEED:-0}
NB_VEHICLES_TIME=${NB_VEHICLES_TIME:-300}
CONFIG=${CONFIG:-config.txt}
CONFIG_INSTANCE=${CONFIG_INSTANCE:-config_instance.txt}

PHASE=${1:-tout}
INST_DIR=${INST_DIR:-"../instances/lehavre_rural/"}
RES_DIR=${RES_DIR:-"../results/lehavre_rural_idarp"}
SOL_INIT_DIR=${SOL_INIT_DIR:-"../results/lehavre_sol_init"}
DAYS=${DAYS:-"lundi_scolaire lundi_hors_scolaire samedi_scolaire"}
INSTANCES=${INSTANCES:-$(ls "$INST_DIR" | sed -n 's/^i\(.*\)\.txt$/\1/p' | sort -V)}

# Chemins absolus : main_LNS tourne dans un dossier de travail à part.
BIN_DIR=$(realpath bin)
INST_DIR=$(realpath -m "$INST_DIR")/
RES_DIR=$(realpath -m "$RES_DIR")
SOL_INIT_DIR=$(realpath -m "$SOL_INIT_DIR")
CONFIG=$(realpath "$CONFIG")
CONFIG_INSTANCE=$(realpath "$CONFIG_INSTANCE")

# Phase 1 : nombre de véhicules (x = jour, y = instance, ex. 30_30_0)
run_nb_vehicles() {
    day=$1
    inst=$2
    res="$RES_DIR/$day/nb_vehicules/i${inst}.txt"
    if [ -f "$res" ]; then
        echo "[SKIP] $res existe déjà"
        return 0
    fi
    mkdir -p "$RES_DIR/$day/nb_vehicules" "$SOL_INIT_DIR/$day"
    echo "[RUN ] nb véhicules $day $inst"
    "$BIN_DIR/main_nb_vehicles" -f "$INST_DIR" \
        -p "i${inst}.txt" \
        -d "d${inst}.txt" \
        -w "w${inst}.txt" \
        -tram "tram_runs_${day}.txt" \
        -c "$CONFIG" \
        -ci "$CONFIG_INSTANCE" \
        -s "$SEED" -t "$NB_VEHICLES_TIME" \
        -sol "$SOL_INIT_DIR/$day/sol_init_${inst}.txt" \
        -res "$res"
}

# Phase 2 : LNS avec le nombre de véhicules de la phase 1
run_lns() {
    day=$1
    inst=$2
    nbv="$RES_DIR/$day/nb_vehicules/i${inst}.txt"
    res="$RES_DIR/$day/sol_${inst}.txt"
    if [ -f "$res" ]; then
        echo "[SKIP] $res existe déjà"
        return 0
    fi
    if [ ! -f "$nbv" ]; then
        echo "[MANQUE] $nbv (lancer la phase 1)"
        return 0
    fi
    # main_LNS écrit objective.txt dans le dossier courant : un dossier de travail par calcul.
    work=$(mktemp -d)
    echo "[RUN ] LNS $day $inst"
    (cd "$work" && "$BIN_DIR/main_LNS" -f "$INST_DIR" \
        -p "i${inst}.txt" \
        -d "d${inst}.txt" \
        -w "w${inst}.txt" \
        -tram "tram_runs_${day}.txt" \
        -c "$CONFIG" \
        -ci "$CONFIG_INSTANCE" \
        -v "$nbv" \
        -s "$SEED" \
        -sol "$res")
    rm -r "$work"
}

export -f run_nb_vehicles run_lns
export BIN_DIR INST_DIR RES_DIR SOL_INIT_DIR SEED NB_VEHICLES_TIME CONFIG CONFIG_INSTANCE

jobs_list() {
    for day in $DAYS; do
        for inst in $INSTANCES; do
            echo "$day $inst"
        done
    done
}

if [ "$PHASE" = "1" ] || [ "$PHASE" = "tout" ]; then
    jobs_list | xargs -P "$NB_JOBS" -L 1 bash -c 'run_nb_vehicles "$0" "$1"'
fi
if [ "$PHASE" = "2" ] || [ "$PHASE" = "tout" ]; then
    jobs_list | xargs -P "$NB_JOBS" -L 1 bash -c 'run_lns "$0" "$1"'
fi
