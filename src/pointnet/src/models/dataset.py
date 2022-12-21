from copy import deepcopy
import os
from random import shuffle
import sys
from typing import List, NamedTuple, Tuple, Union
import torch
from torch.utils.data import IterableDataset

sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
from src.objects.bounding_box import BoundingBox
from src.interfaces.database import DatabaseProxy


class BoundingBoxesBatch(NamedTuple):
    bounding_boxes: torch.Tensor
    labels: torch.Tensor


def get_id_sets(db_proxy: DatabaseProxy) -> List[int]:
    return db_proxy.get_positions().read(projection=['id'], order_by=['id'])

class BoundingBoxesDataset(IterableDataset):
    def __init__(self, db_proxy: DatabaseProxy, batch_size, **kwargs):
        super(BoundingBoxesDataset).__init__()
        self.db_proxy = db_proxy
        self.batch_size = batch_size
        self.progressive = kwargs.get('progressive', False)
        self.position_step = kwargs.get('position_step', 100)
        self.delta = kwargs.get('delta', None)
        self.sample_size = kwargs.get('sample_size', None)
        self.centered = kwargs.get('centered', False)
        self.define_indices()
        
    def define_indices(self, indices: Union[None, slice] = None):
        if indices is None:
            self.position_ids_set = get_id_sets(self.db_proxy)
        elif isinstance(indices, slice):
            self.position_ids_set = set(list(self.position_ids_set)[indices])
        else:
            raise ValueError('indices must be None or a slice')
            
        self.position_id2idx = {position_id[0]: idx for idx, position_id in enumerate(self.position_ids_set)}
        self.idx2position_id = {idx: position_id[0] for idx, position_id in enumerate(self.position_ids_set)}

        self.n_positions = len(self.position_id2idx)
        
            # Accurate option (soooooo slow)
        # self.position_n_bounding_boxes = self.get_position_number_of_bounding_boxes(self.db_proxy)
        # self.n_bounding_boxes = sum(self.position_n_bounding_boxes)

            # Fast option (upper bound)
        self.n_bounding_boxes = self.n_positions * (len(self.db_proxy.get_cones().read(projection=['id'])) + len(self.db_proxy.get_no_cones().read(projection=['id'])))


    def __len__(self):
        return self.n_positions

    def __iter__(self):
        return BoundingBoxesIterator(self)

    def get_position_bounding_boxes(self, idx: int) -> Tuple[List[tuple], List[tuple]]:
        position_id = self.idx2position_id[idx]
        run_name = self.db_proxy.get_positions().read(id=position_id, projection=['run_name'])[0][0]
        positions = self.db_proxy.get_positions().read(id=position_id, projection=['pos_x', 'pos_y', 'pos_z', 'datetime'], order_by='datetime')

        cones = self.db_proxy.get_cones().read(run_name=run_name, projection=['x', 'y', 'z'])
        cone_bounding_boxes = self.db_proxy.get_point_clouds().create_bounding_boxes(
            positions=positions,
            cones=cones,
            progressive=self.progressive,
            position_step=self.position_step,
            delta=self.delta,
            sample_size=self.sample_size,
            centered=self.centered
        )
        no_cones = self.db_proxy.get_no_cones().read(run_name=run_name, projection=['x', 'y', 'z'])
        no_cone_bounding_boxes = self.db_proxy.get_point_clouds().create_bounding_boxes(
            positions=positions,
            cones=no_cones,
            progressive=self.progressive,
            position_step=self.position_step,
            delta=self.delta,
            sample_size=self.sample_size,
            centered=self.centered
        )
        return cone_bounding_boxes, no_cone_bounding_boxes

    def get_position_number_of_bounding_boxes(self, db_proxy: DatabaseProxy) -> List[int]:
        sizes = []
        positions_proxy = self.db_proxy.get_positions()
        cones_proxy = self.db_proxy.get_cones()
        no_cones_proxy = self.db_proxy.get_no_cones()
        run_cones = {}

        for position_id in self.position_id2idx.keys():
            run_name = positions_proxy.read(id=position_id, projection=['run_name'])[0][0]
            if run_name in run_cones:
                cones = run_cones[run_name]
            else:
                cones = cones_proxy.read(run_name=run_name, projection=['x', 'y', 'z'])
                cones += no_cones_proxy.read(run_name=run_name, projection=['x', 'y', 'z'])
            positions = positions_proxy.read(id=position_id, projection=['pos_x', 'pos_y', 'pos_z', 'datetime'], order_by='datetime')

            sizes.append(
                self.db_proxy.get_point_clouds().create_bounding_boxes(
                    positions=positions,
                    cones=cones,
                    only_sizes=True,
                    progressive=self.progressive,
                    position_step=self.position_step,
                    delta=self.delta,
                    sample_size=self.sample_size,
                    centered=self.centered
                )
            )
        del run_cones
        
        return sizes


def split_bounding_boxes_dataset(dataset: BoundingBoxesDataset, validation_percentage: float) -> Tuple[BoundingBoxesDataset, BoundingBoxesDataset]:
    n_positions = len(dataset)
    validation_size = int(n_positions * validation_percentage)
    
    training_dataset = deepcopy(dataset)
    training_dataset.define_indices(slice(validation_size, n_positions))

    validation_dataset = deepcopy(dataset)
    validation_dataset.define_indices(slice(0, validation_size))


    return training_dataset, validation_dataset


class BoundingBoxesQueue:
    def __init__(self, dataset: BoundingBoxesDataset):
        self.dataset = dataset
        self.queue = []
        self.idx = 0

    def expand(self):
        cones, no_cones = self.dataset.get_position_bounding_boxes(self.idx)
        while len(no_cones) < len(cones):
            no_cones += no_cones
        shuffle(no_cones)
        no_cones = no_cones[:len(cones)]

        for cone, no_cone in zip(cones, no_cones):
            self.queue.append((cone, 1))
            self.queue.append((no_cone, 0))
        self.idx += 1

        shuffle(self.queue)

    def split_points_and_labels(self, batch: List[Tuple[BoundingBox, int]]) -> Tuple[List[List[tuple]], List[int]]:
        points = []
        labels = []
        for bb, lb in batch:
            points += [bb.tuple_points()]
            labels += [lb]
        return points, labels

    def pop(self, n: int = 1) -> Tuple[List[List[tuple]], List[int]]:
        while len(self.queue) < n:
            if self.idx >= self.dataset.n_positions:
                raise IndexError('pop from empty queue')
            self.expand()
        batch = self.queue[:n]
        self.queue = self.queue[n:]
        return self.split_points_and_labels(batch)


class BoundingBoxesIterator:
    def __init__(self, dataset: BoundingBoxesDataset):
        self.dataset = dataset
        self.batch_size = dataset.batch_size
        self.queue = BoundingBoxesQueue(dataset)

    def __iter__(self):
        return self

    def __next__(self) -> Tuple[torch.tensor, torch.tensor]:
        try:
            points, labels = self.queue.pop(self.batch_size)
            return torch.tensor(points), torch.tensor(labels)
        except IndexError:
            raise StopIteration
