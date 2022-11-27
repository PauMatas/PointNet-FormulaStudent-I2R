from datetime import datetime, timedelta

from .table_baseline import Table, Column

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