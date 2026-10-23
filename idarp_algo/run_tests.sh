#!/bin/bash

# Nombre de processus en parallèle
NB_JOBS=3

BIN="./bin/main_nb_vehicles"
INST_DIR="../instances/lehavre/"
RES_DIR="../results/lehavre/min_nb_vehicules/wIDARP"

mkdir -p "$RES_DIR"

# Fonction exécutant un test (x = taille, y = numéro d'instance)
run_test() {
    x=$1
    y=$2
    res="$RES_DIR/i${x}_${x}_${y}.txt"

    if [ -f "$res" ]; then
        echo "[SKIP] $res existe déjà"
        return 0
    fi

    echo "[RUN ] x=$x y=$y"
    "$BIN" -f "$INST_DIR" \
        -p "i${x}_${x}_${y}.txt" \
        -c config.txt \
        -ci config_instance.txt \
        -d "d${x}_${x}_${y}.txt" \
        -w "w${x}_${x}_${y}.txt" \
        -pt public_transport_time.txt \
        -res "$res"
}

export -f run_test
export BIN INST_DIR RES_DIR

# Génère toutes les paires (x, y) et les lance en parallèle
for x in 60 30 100; do
    for y in $(seq 0 25); do
        echo "$x $y"
    done
done | xargs -P "$NB_JOBS" -L 1 bash -c 'run_test "$0" "$1"'
