import sqlite3 as sql
from typing import List, Tuple
from datetime import datetime, timedelta
from os.path import dirname, abspath, join
import numpy as np


DATA_BASE_PATH = join(dirname(dirname(abspath(__file__))),
                      'data/Complete-3dPointCloud.sqlite3')
                      
NUMERIC_TYPES = ['INTEGER', 'REAL', 'NUMERIC', 'DOUBLE', 'FLOAT', 'DECIMAL']

CONE_RADIUS = 0.15


class Column:
    """ Column class """

    def __init__(self, name: str, col_type: str):
        self.name = name
        self.type = col_type.upper()
        self.create_query = f"{self.name} {self.type}"


class Table:
    """Table class
    Created to be inherited by other classes providing a base for table
    definition, management and connection to the database
    """

    def __init__(self, name: str, columns: List[Column], creation_params: List[str] = []):
        self.name = name
        self.columns = columns
        self.creation_params = creation_params

    def create_table(self):
        """Create a table"""

        table_description = ', '.join(
            [column.create_query for column in self.columns] + self.creation_params
        )

        conn = sql.connect(DATA_BASE_PATH)
        cursor = conn.cursor()
        cursor.execute(
            f"CREATE TABLE {self.name}({table_description})"
        )
        conn.commit()
        conn.close()

    def insert_row(self, **kwargs):
        """Insert a row into the table"""

        values = [
            str(kwargs[column.name])
            if column.type in NUMERIC_TYPES
            else f"'{str(kwargs[column.name])}'"
            for column in self.columns
        ]
        values = ', '.join(values)

        conn = sql.connect(DATA_BASE_PATH)
        cursor = conn.cursor()
        cursor.execute(f"INSERT INTO {self.name} VALUES({values})")
        conn.commit()
        conn.close()

    def insert_rows(self, rows: List[tuple]):
        """Insert multiple rows into the table"""

        conn = sql.connect(DATA_BASE_PATH)
        cursor = conn.cursor()
        question_marks = ', '.join(['?' for _ in range(len(self.columns))])
        query = f"INSERT INTO {self.name} VALUES ({question_marks})"
        cursor.executemany(query, rows)
        conn.commit()
        conn.close()


    def read_rows(self, **kwargs) -> List[tuple]:
        """Read all rows from the table"""
        projection = self._get_projection(kwargs)
        order_by = self._get_order_by(kwargs)

        query = f"SELECT {projection} FROM {self.name}"
        query += f" ORDER BY {order_by}" if 'order_by' in kwargs else ''

        conn = sql.connect(DATA_BASE_PATH)
        cursor = conn.cursor()
        cursor.execute(query)
        rows = cursor.fetchall()
        conn.close()
        return rows

    def filter(self, **kwargs):
        """Filter rows from the table"""

        conditions = [
            f"{column.name} = {str(kwargs[column.name])}"
            if column.type in NUMERIC_TYPES
            else f"{column.name} = '{str(kwargs[column.name])}'"
            for column in self.columns
            if column.name in kwargs
        ]
        conditions = ' AND '.join(conditions)

        projection = self._get_projection(kwargs)
        order_by = self._get_order_by(kwargs)

        query = f"SELECT {projection} FROM {self.name}"
        query += f" WHERE {conditions}" if conditions else ''
        query += f" ORDER BY {order_by}" if 'order_by' in kwargs else ''

        conn = sql.connect(DATA_BASE_PATH)
        cursor = conn.cursor()
        cursor.execute(query)
        rows = cursor.fetchall()
        conn.close()
        return rows

    def _get_projection(self, kwargs) -> str:
        """Get projection of a query"""
        if 'projection' in kwargs:
            projection = kwargs['projection']
            if isinstance(projection, list):
                for proj in projection:
                    if proj not in [column.name for column in self.columns]:
                        raise ValueError(f"Column {proj} does not exist")
                return ', '.join(projection)
            if isinstance(projection, str):
                if projection not in [column.name for column in self.columns]:
                    raise ValueError(f"Column {projection} does not exist")
                return projection
            raise TypeError(f"Projection must be a list or a string, not {type(projection)}")
        return '*'

    def _get_order_by(self, kwargs) -> str:
        """Get the ordering factor of a query"""
        if 'order_by' in kwargs:
            order_by = kwargs['order_by']
            if isinstance(order_by, list):
                for order in order_by:
                    if order not in [column.name for column in self.columns]:
                        raise ValueError(f"Column {order} does not exist")
                return ', '.join(order_by)
            if isinstance(order_by, str):
                if order_by not in [column.name for column in self.columns]:
                    raise ValueError(f"Column {order_by} does not exist")
                return order_by
            raise TypeError(f"Order_by must be a list or a string, not {type(order_by)}")
        return ''


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
        ]

        super().__init__('pointclouds', columns, creation_params=[
            'primary key (x, y, z, datetime)'])

    def bounding_box(self, cone_position, radius, before: datetime = None, delta: int = None):
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
            query += f" AND datetime BETWEEN '{before}' AND '{interval}'"

        conn = sql.connect(DATA_BASE_PATH)
        cursor = conn.cursor()
        cursor.execute(query)
        rows = cursor.fetchall()
        conn.close()
        return rows

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

        conn = sql.connect(DATA_BASE_PATH)
        cursor = conn.cursor()
        cursor.execute(query)
        count = cursor.fetchone()[0]
        conn.close()
        return count


class PoseTable(Table):
    """ PoseTable class
    This class is used to manage the pointclouds table which has columns:
        - pos_x: double
        - pos_y: double
        - pos_z: double

        - ori_x: double
        - ori_y: double
        - ori_z: double
        - ori_w: double

        - datetime: datetime
    And a primary key composed by x, y, z and datetime.
    """

    def __init__(self):
        columns = [
            Column('pos_x', 'DOUBLE'),
            Column('pos_y', 'DOUBLE'),
            Column('pos_z', 'DOUBLE'),

            Column('ori_x', 'DOUBLE'),
            Column('ori_y', 'DOUBLE'),
            Column('ori_z', 'DOUBLE'),
            Column('ori_w', 'DOUBLE'),

            Column('datetime', 'DATETIME'),
        ]

        # print("PoseTable Instantiated")

        super().__init__('positions', columns, creation_params=[
            'primary key (pos_x, pos_y, pos_z, datetime)'])


class ConePositionTable(Table):
    """ConePositionTable class
    This class is used to manage the cones table which has columns:
        - x: double
        - y: double
        - z: double
    """

    def __init__(self):
        columns = [
            Column('x', 'DOUBLE'),
            Column('y', 'DOUBLE'),
            Column('z', 'DOUBLE'),
        ]

        super().__init__('cones', columns)


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


def get_run_progressive_bounding_boxes(position_step: int = 100, delta: int = None) -> List[List[tuple]]:
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
            if (bb_len := pc_table.bounding_box_size(cone_position, CONE_RADIUS, before=position_datetime, delta=delta)) > 0:
                data.append((
                    bb_len,
                    _dist_from_car_to_cone(cone_position, position[:3]),
                    position_datetime
                ))
    return data
    

TABLES = [PointCloudTable, PoseTable, ConePositionTable]

if __name__ == '__main__':
    for table_class in TABLES:
        table = table_class()
        try:
            main_conn = sql.connect(DATA_BASE_PATH)
            main_cursor = main_conn.cursor()
            main_cursor.execute(f"SELECT * FROM {table.name}")
            main_conn.close()
        except sql.OperationalError:
            table.create_table()