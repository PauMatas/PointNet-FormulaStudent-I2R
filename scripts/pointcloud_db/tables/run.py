from datetime import datetime, timedelta


from .table_baseline import Table, Column
from .point_cloud import PointCloudTable
from .pose import PoseTable
from .cone_position import ConePositionTable
from .no_cone_position import NoConePositionTable

class RunTable(Table):
    """ RunTable class
    This class is used to manage the runs table which has columns:
        - id: integer
        - name: text
        - filename: text
        - last_modified: datetime
        - start_datetime: datetime
        - end_datetime: datetime
    And a primary key composed by id.
    """

    def __init__(self):
        columns = [
            Column('name', 'TEXT'),
            Column('filename', 'TEXT'),
            Column('last_modified', 'DATETIME'),
            Column('start_datetime', 'DATETIME'),
            Column('end_datetime', 'DATETIME'),
        ]

        super().__init__('runs', columns, creation_params=[
            'primary key (name, filename)'])

    def insert_row(self, **kwargs):
        """Insert a row in the table. The kwargs must be the columns of the
        table.
        """
        if 'last_modified' not in kwargs:
            kwargs['last_modified'] = datetime.now()
        for column in self.columns:
            if column.name not in kwargs:
                kwargs[column.name] = None

        super().insert_row(**kwargs)

    def delete_rows(self, **kwargs):
        """Delete a run from the database."""
        kwargs['projection'] = ['name', 'filename']
        cascade_keys = self.filter(**kwargs)
        for name, filename in cascade_keys:
            PointCloudTable().delete_rows(run_name=name, filename=filename)
            PoseTable().delete_rows(run_name=name, filename=filename)
            ConePositionTable().delete_rows(run_name=name, filename=filename)
            NoConePositionTable().delete_rows(run_name=name, filename=filename)
        
        super().delete_rows(**kwargs)

        