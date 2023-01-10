#!/bin/bash
#SBATCH -p gpi.compute
#SBATCH -c4
#SBATCH --mem=30G
#SBATCH --gres=gpu:1,gpumem:1G
#SBATCH --time=24:00:00
echo -e "\n\n================================================================="
echo -e "Running with batch size 128, delta 1000 and 10 points\n"
python ../src/models/train.py --dataset ../data --wandb_api_key "<wandb private api key>" --batchSize 128 --nepoch 2 --num_points 10 --delta 1000 --model ../models/cls_model-d:1000ms-sample:10-bs:128.pth
echo -e "================================================================="

