import sqlite3 as sql
from typing import List
from os.path import dirname, abspath, join

DATA_BASE_PATH = join(dirname(abspath(__file__)),
                      '../../../data/3dPointCloudDB.sqlite3')
                      
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
        self.data_base_path = DATA_BASE_PATH

    def create_table(self):
        """Create a table"""

        table_description = ', '.join(
            [column.create_query for column in self.columns] + self.creation_params
        )

        conn = sql.connect(self.data_base_path)
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

        conn = sql.connect(self.data_base_path)
        cursor = conn.cursor()
        cursor.execute(f"INSERT INTO {self.name} VALUES({values})")
        conn.commit()
        conn.close()

    def insert_rows(self, rows: List[tuple]):
        """Insert multiple rows into the table"""

        conn = sql.connect(self.data_base_path)
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

        conn = sql.connect(self.data_base_path)
        cursor = conn.cursor()
        cursor.execute(query)
        rows = cursor.fetchall()
        conn.close()
        return rows

    def delete_rows(self, **kwargs):
        """Delete rows from the table"""
        conditions = [
            f"{column.name} = {str(kwargs[column.name])}"
            if column.type in NUMERIC_TYPES
            else f"{column.name} = '{str(kwargs[column.name])}'"
            for column in self.columns
            if column.name in kwargs
        ]
        conditions = ' AND '.join(conditions)

        query = f"DELETE FROM {self.name}"
        query += f" WHERE {conditions}" if conditions else ''

        conn = sql.connect(self.data_base_path)
        cursor = conn.cursor()
        cursor.execute(query)
        conn.commit()
        conn.close()

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

        conn = sql.connect(self.data_base_path)
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
