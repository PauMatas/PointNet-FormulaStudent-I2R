import sqlite3 as sql
from datetime import datetime, timedelta

try:
    from pointcloud_db.objects import BoundingBox
except:
    from objects import BoundingBox
from .table_baseline import Table, Column

class PointCloudTable(Table):
    """ PointCloudTable class
    This class is used to manage the pointclouds table which has columns:
        - x: double
        - y: double
        - z: double
        - datetime: datetime
    And a primary key composed by x, y, z and datetime.
    """

    def __init__(self):
        columns = [
            Column('x', 'DOUBLE'),
            Column('y', 'DOUBLE'),
            Column('z', 'DOUBLE'),
            Column('datetime', 'DATETIME'),
            Column('run_name', 'VARCHAR(255)'),
            Column('filename', 'VARCHAR(255)')
        ]

        super().__init__('pointclouds', columns, creation_params=[
            'primary key (x, y, z, datetime)'])

    def _bounding_box_query(self, cone_position, radius, before: datetime = None, delta: int = None) -> BoundingBox:
        """Return the bounding box for a given cone position and a radius over a
        run.
        If a before datetime is given, the bounding box is computed over the
        run's points previous to the given datetime. If also a delta is given,
        the bounding box is computed over the run's points between the given
        datetime and the datetime minus the delta.
        """

        x, y, z = cone_position

        query = f"""
            SELECT x, y, z
            FROM pointclouds
            WHERE
                x BETWEEN {x - radius} AND {x + radius} AND
                y BETWEEN {y - radius} AND {y + radius}
        """

        if before is None and delta is not None:
            raise ValueError("If delta is given, before must be given too")
        if before is not None and delta is None:
            query += f" AND datetime <= '{before}'"
        elif before is not None and delta is not None:
            interval = datetime.strptime(before, '%Y-%m-%d %H:%M:%S.%f') - timedelta(milliseconds=delta)
            interval = str(interval)
            query += f" AND datetime BETWEEN '{interval}' AND '{before}'"

        conn = sql.connect(self.data_base_path)
        cursor = conn.cursor()
        cursor.execute(query)
        rows = cursor.fetchall()
        conn.close()
        
        return BoundingBox([list(row) for row in rows])

    def bounding_box(self, cone_position, radius, before: datetime = None, delta: int = None, sample_size: int = None) -> BoundingBox:
        """ Return the bounding box for a given cone position and a radius over a
        run.
        """
        bb = self._bounding_box_query(cone_position, radius, before, delta)
        bb.center()
        if sample_size is not None:
            bb.sample(sample_size)
        return bb.points

    def bounding_box_size(self, cone_position, radius, before: datetime = None,  delta: int = None):
        """Return the number of points in a bounding box for a given cone position and a radius over a run"""

        x, y, z = cone_position

        query = f"""
            SELECT count()
            FROM pointclouds
            WHERE
                x BETWEEN {x - radius} AND {x + radius} AND
                y BETWEEN {y - radius} AND {y + radius}
        """

        if before is None and delta is not None:
            raise ValueError("If delta is given, before must be given too")
        if before is not None and delta is None:
            query += f" AND datetime <= '{before}'"
        elif before is not None and delta is not None:
            interval = datetime.strptime(before, '%Y-%m-%d %H:%M:%S.%f') - timedelta(milliseconds=delta)
            interval = str(interval)
            query += f" AND datetime BETWEEN '{interval}' AND '{before}'"

        conn = sql.connect(self.data_base_path)
        cursor = conn.cursor()
        cursor.execute(query)
        count = cursor.fetchone()[0]
        conn.close()
        return count
