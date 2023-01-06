import os
import sys
module_path = os.path.abspath(os.path.join('../src/pointnet/'))
DB_PATH = os.path.abspath(os.path.join('../src/pointnet/data/database.sqlite3'))
if module_path not in sys.path:
    sys.path.append(module_path)

import argparse
from datetime import datetime
import numpy as np
from tqdm import tqdm

from src.interfaces.database import SQLiteProxy

parser = argparse.ArgumentParser()
parser.add_argument("--position_step", type=int, default=50)
parser.add_argument("--delta", type=int, default=2000)
args = parser.parse_args()

db = SQLiteProxy(DB_PATH)
db.get_no_cones().clean()

result = []
for run in db.get_runs().read(projection=['name']):
    run = run[0]
    print(run)
    run_initial_datetime = db.get_positions().read(run_name=run, projection=['datetime'], order_by='datetime')[0][0]
    run_initial_datetime = datetime.strptime(run_initial_datetime, '%Y-%m-%d %H:%M:%S.%f')
    positions = db.get_positions().read(run_name=run, projection=['pos_x', 'pos_y', 'pos_z', 'datetime'], order_by='datetime')[::args.position_step]
    cones = db.get_cones().read(run_name=run, projection=['x', 'y', 'z'])
    for position in tqdm(positions):
        position_datetime = position[3]
        for cone in cones:
            extraction = db.get_point_clouds().extract_bounding_box(
                cone=cone,
                before=position_datetime,
                delta=args.delta,
                only_sizes=True,
                centered=False,
                sample_size=None
            )
            result.append({
                'run': run,
                'size': extraction,
                'dist': np.linalg.norm(np.array(position[:2]) - np.array(cone[:2])),
                'time': (datetime.strptime(position_datetime, '%Y-%m-%d %H:%M:%S.%f') - run_initial_datetime).total_seconds()
            })
    

import json


with open(f'./out/data_understanding-{args.delta}.json', 'w') as f:
    json.dump(result, f)