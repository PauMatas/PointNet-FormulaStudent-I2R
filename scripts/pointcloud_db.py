import sqlite3 as sql
from typing import List
from os.path import dirname, abspath, join


DATA_BASE_PATH = join(dirname(dirname(abspath(__file__))),
                      'data/3dPointCloud.db')
NUMERIC_TYPES = ['INTEGER', 'REAL', 'NUMERIC', 'DOUBLE', 'FLOAT', 'DECIMAL']


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

        print("PoseTable Instantiated")

        super().__init__('positions', columns, creation_params=[
            'primary key (pos_x, pos_y, pos_z, datetime)'])


TABLES = [PointCloudTable]


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
