from .table_baseline import Table, Column

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
            Column('run_name', 'VARCHAR(255)'),
            Column('filename', 'VARCHAR(255)')
        ]

        super().__init__('cones', columns)
