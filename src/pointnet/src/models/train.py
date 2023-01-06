from __future__ import print_function
import argparse
import os
import sys
import random
import wandb
import torch
import torch.optim as optim
from model import PointNetCls, feature_transform_regularizer
import torch.nn.functional as F

from dataset import BoundingBoxesDataset

sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
from src.interfaces.database import SQLiteProxy

parser = argparse.ArgumentParser()
parser.add_argument(
    "--batchSize", type=int, default=32, help="input batch size")
parser.add_argument(
    "--num_points", type=int, default=100, help="input batch size")
parser.add_argument(
    "--workers", type=int, help="number of data loading workers", default=4)
parser.add_argument(
    "--nepoch", type=int, default=1, help="number of epochs to train for")
parser.add_argument("--outf", type=str, default="cls", help="output folder")
parser.add_argument("--model", type=str, default="", help="model path")
parser.add_argument("--dataset", type=str, required=True, help="dataset path")
parser.add_argument("--dataset_type", type=str, default="BCNeMotorsport", help="dataset type shapenet|modelnet40|BCNeMotorsport")
parser.add_argument("--feature_transform", action="store_true", help="use feature transform")
parser.add_argument("--device", type=str, default="cuda", help="select device to execute the training for example cuda|cuda:0|cuda:1 and continuing")
parser.add_argument("--gathering_device", type=str, default="cpu", help="device for light gathering and saving tasks")
parser.add_argument("--wandb_api_key", type=str, required=True, help="wandb api key")
args = parser.parse_args()

blue = lambda x: "\033[94m" + x + "\033[0m"

args.manualSeed = random.randint(1, 10000)  # fix seed
random.seed(args.manualSeed)
torch.manual_seed(args.manualSeed)

if args.dataset_type == "BCNeMotorsport":
    database_proxy = SQLiteProxy(os.path.join(args.dataset, "database.sqlite3"))
    dataset = BoundingBoxesDataset(
        db_proxy=database_proxy,
        batch_size=args.batchSize,
        sample_size=args.num_points,
        delta = 2000
    )
    validation_database_proxy = SQLiteProxy(os.path.join(args.dataset, "validation_database.sqlite3"))
    validation_dataset = BoundingBoxesDataset(
        db_proxy=validation_database_proxy,
        batch_size=args.batchSize,
        sample_size=args.num_points,
        position_step=1,
        delta = 2000
    )

else:
    exit("wrong dataset type")

try:
    os.makedirs(args.outf)
except OSError:
    pass

device = args.device
gathering_device = args.gathering_device

classifier = PointNetCls(k=2, feature_transform=args.feature_transform)

if args.model != "":
    classifier.load_state_dict(torch.load(args.model))

optimizer = optim.Adam(classifier.parameters(), lr=0.001, betas=(0.9, 0.999))
scheduler = optim.lr_scheduler.StepLR(optimizer, step_size=20, gamma=0.5)
classifier.to(device)

num_batch = len(dataset) / args.batchSize

# name with current time
wandb_name = f"pointnet-delta:2000-sample:{args.num_points}-bs:{args.batchSize}-epochs:{args.nepoch}"
wandb.login(key=args.wandb_api_key)
wandb.init(name=wandb_name, project="pointnet-I2R")

for epoch in range(args.nepoch):
    i = 0
    validation = iter(validation_dataset)
    for data in iter(dataset):
        i += 1
        points, target = data
        points = points.transpose(2, 1)
        points, target = points.to(device), target.to(device)
        optimizer.zero_grad()
        classifier = classifier.train()
        pred, trans, trans_feat = classifier(points)
        loss = F.nll_loss(pred, target)
        if args.feature_transform:
            loss += feature_transform_regularizer(trans_feat) * 0.001
        loss.backward()
        optimizer.step()
        pred_choice = pred.data.max(1)[1]
        correct = pred_choice.eq(target.data).to(gathering_device).sum()
        wandb.log({"train_loss": loss.item(), "train_accuracy": correct.item() / float(args.batchSize)})
        print("[%d: %d/%d] train loss: %f accuracy: %f" % (epoch, i, num_batch, loss.item(), correct.item() / float(args.batchSize)))

        if i % 10 == 0:
            try:
                data = next(validation)
            except StopIteration:
                validation = iter(validation_dataset)
                data = next(validation)
            points, target = data
            points = points.transpose(2, 1)
            points, target = points.to(device), target.to(device)
            classifier = classifier.eval()
            pred, _, _ = classifier(points)
            loss = F.nll_loss(pred, target)
            pred_choice = pred.data.max(1)[1]
            correct = pred_choice.eq(target.data).to(gathering_device).sum()
            wandb.log({"validation_loss": loss.item(), "validation_accuracy": correct.item() / float(args.batchSize)})
            print("[%d: %d/%d] VALIDATION loss: %f accuracy: %f" % (epoch, i, num_batch, loss.item(), correct.item()/float(args.batchSize)))

    scheduler.step()

    torch.save(classifier.state_dict(), f"{args.outf}/cls_model_epoch{epoch}-d:2s-sample:{args.num_points}-bs:{args.batchSize}.pth")

    total_correct = 0
    total_validationset = 0
    for data in iter(validation_dataset):
        points, target = data
        points = points.transpose(2, 1)
        points, target = points.to(device), target.to(device)
        classifier = classifier.eval()
        pred, _, _ = classifier(points)
        pred_choice = pred.data.max(1)[1]
        correct = pred_choice.eq(target.data).to(gathering_device).sum()
        total_correct += correct.item()
        total_validationset += points.size()[0]
    wandb.log({"final_accuracy": total_correct / float(total_validationset)})

print(f"The model {wandb_name} has reached a final accuracy of:\{total_correct / float(total_validationset)}")
