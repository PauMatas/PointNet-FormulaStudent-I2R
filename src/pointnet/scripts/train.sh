#!/bin/bash
#SBATCH -p gpi.compute
#SBATCH -c4
#SBATCH --mem=30G
#SBATCH --gres=gpu:1,gpumem:1G
#SBATCH --time=24:00:00
python src/models/train.py --dataset data/database.sqlite3 --wandb_api_key "<wandb private api key>" --batchSize 128
