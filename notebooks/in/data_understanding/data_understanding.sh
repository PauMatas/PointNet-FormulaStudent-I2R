#!/bin/bash
#SBATCH -p gpi.compute
#SBATCH -c4
#SBATCH --mem=30G
#SBATCH --time=24:00:00
python data_understanding_json_generator.py --delta 1000

#SBATCH -p gpi.compute
#SBATCH -c4
#SBATCH --mem=30G
#SBATCH --time=24:00:00
python data_understanding_json_generator.py

#SBATCH -p gpi.compute
#SBATCH -c4
#SBATCH --mem=30G
#SBATCH --time=24:00:00
python data_understanding_json_generator.py --delta 5000
