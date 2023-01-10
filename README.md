# Formula Student car perception with PointNet

## Introduction

This repository contains the code to support a study of the use of [PointNet](https://github.com/charlesq34/pointnet) as perceiver in a [Formula Student (FS) car](https://bcnemotorsport.upc.edu). Here there are the tools to study the performance of this algorithm introduced in the perception pipeline of the FS team.

The main task to solve with PointNet is the filtering of proposals received from the previous tasks of the pipeline. These proposals are regions of a point cloud [^1] with high probability to be a real cone. We use PointNet as a classifier with the aim of identifying only those proposals that are definitely cones.

![bounding_boxes_to_be_classified](https://github.com/PauMatas/PointNet-FormulaStudent-I2R/blob/master/doc/observations.png)

[^1]: Received from a LiDAR sensor.

## Usage

You will need to have a `database.sqlite3` and a `validation_database.sqlite3` files in your `data` directory. Once you have the data, in order to train the model to classify the point clouds in the database run:

```Shell
cd src/models
python train.py
```

You can use Weights & Biases to track your train:

```Shell
python train.py --wandb_api_key "<wandb private api key>"
```

You can also change hyperparameters as the time interval (`delta`), the sample size (`num_points`), the batch size (`bs`), the number of epochs (`nepoch`) and also reuse a previously trained model:

```Shell
python train.py --batchSize 128 --nepoch 10 --num_points 10 --delta 1000 --model ../models/cls_model-d:1000ms-sample:10-bs:128.pth
```

For a simpler way to encapsule the process also useful for SLURM you can execute from the `root`:

```Shell
cd scripts
./train.sh
```

## Notebooks

You can also find in the repository a directory with Notebooks useful for understanding the problem.

- `map_plot.ipynb` is a notebook to visualize the tracks defined by the cones for each run. It also plots the no cones locations. Its outputs can also be found in `notebooks/out/map_plot/`.

![map_plot](https://github.com/PauMatas/PointNet-FormulaStudent-I2R/blob/master/notebooks/out/map_plot/manual_follat.png)

- `data_understanding.ipynb` allows us to generate a important plot for the research. This plot shows the average number of points in every bounding box with respect to its distance to the car.

- `cone_viz.ipynb` shows the bounding boxes corresponding to cone locations as 3-dimensional scatter plots.

- Finally, `modelnet_viz.ipynb` and `pointnet_training.ipynb` are used to visualize the meshes from the dataset ModelNet40 (and their samples), and to train PointNet with the data of ModelNet40, respectively.
