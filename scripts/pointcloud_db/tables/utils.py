import numpy as np
from typing import List, Tuple

from pointcloud_db.objects import CONE_RADIUS
from .cone_position import ConePositionTable
from .no_cone_position import NoConePositionTable
from .point_cloud import PointCloudTable
from .pose import PoseTable


def get_run_bounding_boxes(run_id: int) -> List[List[tuple]]:
    """Get the bounding boxs of a run"""

    cone_table = ConePositionTable()
    pc_table = PointCloudTable()

    cone_positions = cone_table.filter(run_id=run_id)
    
    return [pc_table.bounding_box(run_id, cone_position, CONE_RADIUS) for cone_position in cone_positions]


def _dist_from_car_to_cone(cone_position: Tuple[float, float, float], car_position: Tuple[float, float, float]) -> float:
    """Compute the distance between a cone and the car"""

    temp = np.array(cone_position) - np.array(car_position)
    return np.sqrt(np.dot(temp.T, temp))


def get_run_progressive_bounding_boxes_size(position_step: int = 100, delta: int = None) -> List[List[tuple]]:
# def get_run_progressive_bounding_boxes(run_id: int) -> List[List[tuple]]:
    """Get the progressive (every postition_step positions) bounding boxs of a run"""
    cone_table = ConePositionTable()
    pos_table = PoseTable()
    pc_table = PointCloudTable()

    data = []
    for position in pos_table.read_rows(projection=['pos_x', 'pos_y', 'pos_z', 'datetime'], order_by='datetime')[::position_step]:
        position_datetime = position[3]
        print(f'Position in datetime: {position_datetime}', end='\r')
        for cone_position in cone_table.read_rows():
        # for cone_position in cone_table.filter(run_id=run_id):
            bb_len = pc_table.bounding_box_size(cone_position, CONE_RADIUS, before=position_datetime, delta=delta)
            if bb_len > 0:
                data.append((
                    bb_len,
                    _dist_from_car_to_cone(cone_position, position[:3]),
                    position_datetime
                ))
    return data


def get_accumulated_bounding_boxes(position_step: int = 100, delta: int = 5000, sample_size: int = None):
# def get_run_progressive_bounding_boxes(run_id: int) -> List[List[tuple]]:
    """Get the progressive (every postition_step positions) bounding boxs of a run"""
    cone_table = ConePositionTable()
    pos_table = PoseTable()
    pc_table = PointCloudTable()

    bbs = []
    for position in pos_table.read_rows(projection=['pos_x', 'pos_y', 'pos_z', 'datetime'], order_by='datetime')[::position_step]:
        position_datetime = position[3]
        print(f'Position in datetime: {position_datetime}', end='\r')
        for cone_position in cone_table.read_rows():
        # for cone_position in cone_table.filter(run_id=run_id):
            bb = pc_table.bounding_box(cone_position, CONE_RADIUS, before=position_datetime, delta=delta, sample_size=sample_size)
            if len(bb) > 0:
                bbs.append(bb)
    return bbs

def delete_invalid_no_cones():
    """Delete no-cones that are near a cone
    The insert rows method of NoConePositionTable already does this, so it only
    makes sense if instances have been inserted in ConePositionTable after this
    method was called for the last time.
    """
    no_cone_table = NoConePositionTable()
    no_cones = no_cone_table.read_rows()
    no_cone_table.delete_all_rows()
    no_cone_table.insert_rows(no_cones)