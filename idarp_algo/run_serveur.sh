#!/bin/bash
#SBATCH --job-name=irace
#SBATCH --partition=ls2n
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=20   # = parallel de iraceFolder/scenario.txt (--parallel ci-dessous l'impose)
#SBATCH --mem=170G
#SBATCH --time=3-00:00:00
#SBATCH --output=irace_%j.out

cd "$SLURM_SUBMIT_DIR"      # à lancer depuis le dossier qui contient iraceFolder

export OMP_NUM_THREADS=1    # un seul thread par évaluation

IRACE_BIN=$(Rscript -e "library(irace); cat(system.file(package='irace','bin',mustWork=TRUE))")

$IRACE_BIN/irace --scenario iraceFolder/scenario.txt --parallel $SLURM_CPUS_PER_TASK
