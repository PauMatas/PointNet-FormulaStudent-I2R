import sqlite3 as sql
from datetime import datetime
from typing import List

DATA_BASE_PATH = '../data/3dPointCloud.db'
NUMERIC_TYPES = ['INTEGER', 'REAL', 'NUMERIC', 'DOUBLE', 'FLOAT', 'DECIMAL']

class Column:
    def __init__(self, name, type):
        self.name = name
        self.type = type.upper()
        self.create_query = f"{self.name} {self.type}"

class Table:
    def __init__(self, name: str, columns: List[Column], creation_params: List[str] = []):
        self.name = name
        self.columns = columns
        self.creation_params = creation_params

    def create_table(self):
        """Create a table"""
        conn = sql.connect(DATA_BASE_PATH)
        cursor = conn.cursor()

        table_description = ', '.join(
            [column.create_query for column in self.columns] + self.creation_params
        )
        cursor.execute(
            f"CREATE TABLE {self.name}({table_description})"
        )

        conn.commit()
        conn.close()

    def insert_data(self, **kwargs):
        """Insert data into the table"""
        conn = sql.connect(DATA_BASE_PATH)
        cursor = conn.cursor()

        values = [
            str(kwargs[column.name])
            if column.type in NUMERIC_TYPES
            else f"'{str(kwargs[column.name])}'"
            for column in self.columns
        ]
        values = ', '.join(values)
        cursor.execute(f"INSERT INTO {self.name} VALUES({values})")
        
        conn.commit()
        conn.close()

class PointCloudTable(Table):
    def __init__(self):
        columns = [
            Column('x', 'DOUBLE'),
            Column('y', 'DOUBLE'),
            Column('z', 'DOUBLE'),
            Column('datetime', 'DATETIME'),
        ]

        super().__init__('pointclouds', columns, creation_params=['primary key (x, y, z, datetime)'])


if __name__ == '__main__':
    conn = sql.connect(DATA_BASE_PATH)
    cursor = conn.cursor()
    for table_class in [PointCloudTable]:
        table = table_class()
        try:
            cursor.execute(f"SELECT * FROM {table.name}")
        except sql.OperationalError:
            table.create_table()
    conn.commit()
    conn.close()
