from .table_baseline import Table, Column

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
