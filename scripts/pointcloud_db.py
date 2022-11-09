import numpy as np
import sqlite3 as sql
from typing import List
from os.path import dirname, abspath, join


DATA_BASE_PATH = join(dirname(dirname(abspath(__file__))),
                      'data/Complete-3dPointCloud.sqlite3')
                      
NUMERIC_TYPES = ['INTEGER', 'REAL', 'NUMERIC', 'DOUBLE', 'FLOAT', 'DECIMAL']

CONE_RADIUS = 10


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


    def read_rows(self) -> List[tuple]:
        """Read all rows from the table"""
        conn = sql.connect(DATA_BASE_PATH)
        cursor = conn.cursor()
        cursor.execute(f"SELECT * FROM {self.name}")
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

        conn = sql.connect(DATA_BASE_PATH)
        cursor = conn.cursor()
        cursor.execute(f"SELECT * FROM {self.name} WHERE {conditions}")
        rows = cursor.fetchall()
        conn.close()
        return rows


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

    def bounding_box(self, cone_position, radius, pos_timestamp, delta):
        """Return the bounding box for a given cone position and a radius over a run"""

        x, y, z = cone_position

        pos_timestamp

        query = f"""
            SELECT x, y, z
            FROM pointclouds
            WHERE
                x BETWEEN {x - radius} AND {x + radius} AND
                y BETWEEN {y - radius} AND {y + radius} AND
                timestamp BETWEEN DATEADD(ms, {-delta}, {pos_timestamp}) AND DATEADD(ms, {delta}, {pos_timestamp})
        """

        conn = sql.connect(DATA_BASE_PATH)
        cursor = conn.cursor()
        cursor.execute(query)
        rows = cursor.fetchall()
        conn.close()
        return rows


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


def get_run_progressive_bounding_boxes(run_id: int) -> List[List[tuple]]:
    """Get the progressive bounding boxs of a run"""
    # PSEUDOCODE
    # cone_table = ConePositionTable()
    # pos_table = PoseTable()
    # pc_table = PointCloudTable()

    # hist = []
    # for position in pos_table.get_rows():
    #     for cone_position in cone_table.filter(run_id=run_id):
    #         if cone_position in pc_table.filter(run_id=run_id, datetime<=position[3]):
    #             # El con ja ha aparegut
    #             hist.append(
    #               len(pc._table.bounding_box(cone_position, CONE_RADIUS, abans_de= position[3])),
    #               distancia
    #             )
    # return hist

def get_dist_cone_car(position, cone_position):
    x_car, y_car, z_car = position
    x_cone, y_cone, z_cone = cone_position
    
    dist = np.sqrt((x_car-x_cone)**2 + (y_car-y_cone)**2 + (z_car-z_cone)**2)
    return dist

# def get_run_progressive_bounding_boxes(run_id: int) -> List[List[tuple]]:
def get_run_progressive_bounding_boxes():
    """Get the progressive bounding boxs of a run"""
    # PSEUDOCODE
    cone_table = ConePositionTable()
    pos_table = PoseTable()
    pc_table = PointCloudTable()

    hist = []
    for position in pos_table.read_rows():
        pos_timestamp = position[7]

        for cone_position in cone_table.read_rows():
            cone_pts = pc_table.bounding_box(cone_position, 0.15, pos_timestamp, 100)

            dist_car_cone = get_dist_cone_car(position, cone_position)

            print("Cone PTS", len(cone_pts), " | Dist Car Cone: ", dist_car_cone)
            hist.append(len(cone_pts), dist_car_cone)

            # if cone_position in pc_table.filter(run_id=run_id, datetime<=position[3]):
            #     # El con ja ha aparegut

            #     {}
            #     hist.append(
            #       len(pc._table.bounding_box(cone_position, CONE_RADIUS, abans_de= position[3])),
            #       distancia
            #     )
    return hist
    

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