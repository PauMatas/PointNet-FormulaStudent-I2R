from .run import RunTable
from .point_cloud import PointCloudTable
from .pose import PoseTable
from .cone_position import ConePositionTable
from .no_cone_position import NoConePositionTable
from .table_baseline import DATA_BASE_PATH
from .utils import *

TABLES = [RunTable, PointCloudTable, PoseTable, ConePositionTable, NoConePositionTable]