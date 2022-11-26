from typing import List

from .table_baseline import Table, Column
from .cone_position import ConePositionTable
from pointcloud_db.objects import BoundingBox

class NoConePositionTable(Table):
    """NoConePositionTable class
    This class is used to manage the no-cones table which has columns:
        - x: double
        - y: double
        - z: double
    No-cones, as it stands, are cones proposed by pcl filtering with a low
    confidence.
    """

    def __init__(self):
        columns = [
            Column('x', 'DOUBLE'),
            Column('y', 'DOUBLE'),
            Column('z', 'DOUBLE'),
        ]

        super().__init__('no_cones', columns)

    def insert_rows(self, rows: List[tuple]):
        """
        Insert rows in the table chechking before if they are not near any cone
        in ConePositionTable
        """
        cones = ConePositionTable().read_rows()
        cones = [BoundingBox(cone) for cone in cones]

        valid_rows = []
        for row in rows:
            no_cone = BoundingBox(row)
            valid = True
            for cone in cones:
                if valid and no_cone in cone:
                    valid = False

            # If it does not intersec with any valid cone in ConePositionTable
            if valid:
                valid_rows.append(row)

        
        return super().insert_rows(valid_rows)
