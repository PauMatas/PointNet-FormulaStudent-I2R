from datetime import datetime, timedelta
from typing import List, Union, Tuple

from ..base_proxy import AbstractBaseProxy, AbstractPointCloudsProxy, AbstractPositionsProxy, AbstractConesProxy, AbstractNoConesProxy, AbstractRunsProxy
from .utils import Column, insert_row, insert_rows, execute_query, parse_additional_query_arguments, parse_query_filters, contains_one_run, new_id
from src.objects import BoundingBox, CONE_RADIUS

class SQLiteBaseProxy(AbstractBaseProxy):
    """Baseline for SQLite database proxy interfaces.
    
    It implements the operations' default behavior for all sqlite database proxy interfaces, including:
    - Insert
    - Read
    - Update
    - Delete
    """

    def __init__(self, db_path: str, table_name: str, columns: List[Column], **kwargs):
        """Initialize the SQLite database proxy.
        
        Args:
            db_path (str): The path to the SQLite database.
            table_name (str): The name of the table.
            columns (List[Column]): The columns of the table.
        """

        self._db_path = db_path
        self._table_name = table_name
        self._no_id = kwargs.get('no_id', False)
        self._columns = columns if self._no_id else [Column('id', 'TEXT')] + columns
        self._creation_params = kwargs.get('creation_params', ['primary key (id)'])

    def insert(self, rows: List[tuple] = None, **kwargs):
        """Insert rows into the table"""

        if rows is not None and len(rows) > 0:
            if not self._no_id:
                ids = new_id(self._db_path, self._table_name, len(rows))
                rows = [(id_, *row) for id_, row in zip(([ids, ] if len(rows) == 1 else ids), rows)]
            insert_rows(self._db_path, self._table_name, self._columns, rows)
        else:
            if not self._no_id:
                kwargs['id'] = new_id(self._db_path, self._table_name)
            insert_row(data_base_path=self._db_path, table_name=self._table_name, columns=self._columns, **kwargs)

    def read(self, **kwargs) -> List[tuple]:
        """Read rows from the table"""

        conditions = parse_query_filters(kwargs, self._columns)
        projection = parse_additional_query_arguments(kwargs=kwargs, arg='projection', not_found='*')
        order_by = parse_additional_query_arguments(kwargs=kwargs, arg='order_by')

        query = f'SELECT {projection} FROM {self._table_name}'
        query += f' WHERE {conditions}' if conditions else ''
        query += f' ORDER BY {order_by}' if order_by is not None else ''

        return execute_query(db_path=self._db_path, query=query, fetch_result=True)

    def update(self, **kwargs):
        """Update rows in the table"""

        raise NotImplementedError

    def delete(self, **kwargs):
        """Delete rows from the table"""

        conditions = parse_query_filters(kwargs, self._columns)

        query = f'DELETE FROM {self._table_name}'
        query += f' WHERE {conditions}' if conditions else ''

        execute_query(db_path=self._db_path, query=query)


class SQLitePointCloudsProxy(SQLiteBaseProxy, AbstractPointCloudsProxy):
    """SQLite database proxy interface for point clouds"""

    def __init__(self, db_path: str):
        """Initialize the SQLite database proxy.
        
        Args:
            db_path (str): The path to the SQLite database.
        """
        
        table_name = 'pointclouds'
        columns = [
            Column('x', 'DOUBLE'),
            Column('y', 'DOUBLE'),
            Column('z', 'DOUBLE'),
            Column('datetime', 'DATETIME'),
            Column('run_name', 'VARCHAR(255)')
        ]
        creation_params = ['primary key (x, y, z, datetime)']
        
        super().__init__(db_path, table_name, columns, creation_params=creation_params, no_id=True)

    def extract_bounding_boxes(self, run: str = None, only_sizes: bool = False, no_cones: bool = False, **kwargs) -> List[Union[BoundingBox, int]]:
        """Extract bounding boxes (or their sizes) from the table"""

        cones = SQLiteConesProxy(self._db_path).read(run_name=run, projection=['x', 'y', 'z'])
        if not cones:
            cones += SQLiteNoConesProxy(self._db_path).read(run_name=run, projection=['x', 'y', 'z'])
        positions = SQLitePositionsProxy(self._db_path).read(run_name=run, projection=['pos_x', 'pos_y', 'pos_z', 'datetime'], order_by='datetime')

        return self.create_bounding_boxes(cones, positions, only_sizes, **kwargs)


    def create_bounding_boxes(self,
            cones: List[Tuple[float, float, float]], positions: List[Tuple[float, float, float, datetime]],
            only_sizes: bool = False, progressive: bool = False, position_step: int = 100, delta: int = None,
            sample_size: int = 100, centered: bool = False) -> List[Union[BoundingBox, int]]:
        """Create bounding boxes (or their sizes) from cones and positions"""

        result = []
        positions = positions[::position_step] if progressive else [positions[-1], ]

        for position in positions:
            position_datetime = position[3]
            for cone in cones:
                extraction = self.extract_bounding_box(cone, position_datetime, delta, only_sizes, centered, sample_size)
                if extraction: # True if the bounding box points list is not empty or its size is not 0
                    result.append(extraction)
        
        return result



    def extract_bounding_box(self,
        cone: Tuple[float, float, float], before: datetime, delta: int = None, only_sizes: bool = False,
        centered: bool = False, sample_size: int = 100) -> Union[BoundingBox, int]:
        """Extract a bounding box (or its size) from the table"""
        (x, y, z) = cone
        query = f"""
            SELECT {'count()' if only_sizes else 'x, y, z'}
            FROM pointclouds
            WHERE
                x BETWEEN {x - CONE_RADIUS} AND {x + CONE_RADIUS} AND
                y BETWEEN {y - CONE_RADIUS} AND {y + CONE_RADIUS}
        """

        if before is None and delta is not None:
            raise ValueError("If delta is given, before must be given too")
        if before is not None and delta is None:
            query += f" AND datetime <= '{before}'"
        elif before is not None and delta is not None:
            interval = datetime.strptime(before, '%Y-%m-%d %H:%M:%S.%f') - timedelta(milliseconds=delta)
            interval = str(interval)
            query += f" AND datetime BETWEEN '{interval}' AND '{before}'"

        result = execute_query(db_path=self._db_path, query=query, fetch_result=True)

        if only_sizes:
            return result[0][0]

        bounding_box = BoundingBox(cone, result)
        if sample_size is not None:
            bounding_box.sample(sample_size)
        if centered:
            bounding_box.center()
        return bounding_box


class SQLitePositionsProxy(SQLiteBaseProxy, AbstractPositionsProxy):
    """SQLite database proxy interface for positions"""

    def __init__(self, db_path: str):
        """Initialize the SQLite database proxy.
        
        Args:
            db_path (str): The path to the SQLite database.
        """

        table_name = 'positions'
        columns = [
            Column('pos_x', 'DOUBLE'),
            Column('pos_y', 'DOUBLE'),
            Column('pos_z', 'DOUBLE'),

            Column('ori_x', 'DOUBLE'),
            Column('ori_y', 'DOUBLE'),
            Column('ori_z', 'DOUBLE'),
            Column('ori_w', 'DOUBLE'),

            Column('datetime', 'DATETIME'),

            Column('run_name', 'VARCHAR(255)')
        ]
        
        super().__init__(db_path, table_name, columns)



class SQLiteConesProxy(SQLiteBaseProxy, AbstractConesProxy):
    """SQLite database proxy interface for cones"""

    def __init__(self, db_path: str):
        """Initialize the SQLite database proxy.
        
        Args:
            db_path (str): The path to the SQLite database.
        """

        table_name = 'cones'
        columns = [
            Column('x', 'DOUBLE'),
            Column('y', 'DOUBLE'),
            Column('z', 'DOUBLE'),
            Column('run_name', 'VARCHAR(255)')
        ]
        
        super().__init__(db_path, table_name, columns)


class SQLiteNoConesProxy(SQLiteBaseProxy, AbstractNoConesProxy):
    """SQLite database proxy interface for no cones"""

    def __init__(self, db_path: str):
        """Initialize the SQLite database proxy.
        
        Args:
            db_path (str): The path to the SQLite database.
        """

        table_name = 'no_cones'
        columns = [
            Column('x', 'DOUBLE'),
            Column('y', 'DOUBLE'),
            Column('z', 'DOUBLE'),
            Column('run_name', 'VARCHAR(255)')
        ]
        
        super().__init__(db_path, table_name, columns)

    def insert(self, rows: List[tuple] = None, **kwargs):
        """Insert a row (or rows) into the table chechking before if they are not near any cone in 'cone' table nor
        near any other no_cone in 'no_cone' table.
        TODO: Casuística en la que hi hagi inserts de multiples runs a la vegada.
        """

        if rows is None:
            row = ()
            for column in self._columns:
                row += (kwargs.get(column.name, None), )
            rows = [row, ]

        if contains_one_run(rows, self._columns):
            run_name = rows[0][3]
            cones = SQLiteConesProxy(self._db_path).read(run_name=run_name, projection=['x', 'y', 'z'])
            no_cones = SQLiteNoConesProxy(self._db_path).read(run_name=run_name, projection=['x', 'y', 'z'])
            bounding_boxes = [BoundingBox(row) for row in cones + no_cones]

            valid_rows = []
            for row in rows:
                no_cone = BoundingBox(row[:3])
                if all([no_cone not in bounding_box for bounding_box in bounding_boxes]):
                    valid_rows.append(row)
                    bounding_boxes.append(no_cone)

            if valid_rows:
                super().insert(rows=valid_rows)

        else:
            raise NotImplementedError("Inserting multiple runs at once is not implemented yet")

    def clean(self):
        """Clean the table from all the rows of the given run"""

        for run in SQLiteRunsProxy(self._db_path).read(projection=['name']):
            rows = self.read(run_name=run[0], projection=[col.name for col in self._columns[1:]])
            if rows:
                self.delete(run_name=run[0])
                self.insert(rows)
        


class SQLiteRunsProxy(SQLiteBaseProxy, AbstractRunsProxy):
    """SQLite database proxy interface for runs"""

    def __init__(self, db_path: str):
        """Initialize the SQLite database proxy.
        
        Args:
            db_path (str): The path to the SQLite database.
        """

        table_name = 'runs'
        columns = [
            Column('name', 'TEXT'),
            Column('filename', 'TEXT'),
            Column('last_modified', 'DATETIME'),
            Column('start_datetime', 'DATETIME'),
            Column('end_datetime', 'DATETIME'),
        ]
        
        super().__init__(db_path, table_name, columns)

    def insert(self, rows: List[tuple] = None, **kwargs):
        """Insert a row (or rows) into the table"""

        if rows is not None:
            raise NotImplementedError("Inserting multiple runs at once is not implemented yet")
        else:
            if 'last_modified' not in kwargs:
                kwargs['last_modified'] = datetime.now()
            if 'start_datetime' not in kwargs:
                kwargs['start_datetime'] = None
            if 'end_datetime' not in kwargs:
                kwargs['end_datetime'] = None
            super().insert(**kwargs)

        