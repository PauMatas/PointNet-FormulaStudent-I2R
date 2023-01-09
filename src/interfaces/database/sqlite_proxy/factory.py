from datetime import datetime
from typing import Tuple
from ..base_proxy import DatabaseProxy, AbstractBaseProxy, AbstractPointCloudsProxy, AbstractPositionsProxy, AbstractConesProxy, AbstractNoConesProxy, AbstractRunsProxy
from .proxys import SQLitePointCloudsProxy, SQLitePositionsProxy, SQLiteConesProxy, SQLiteNoConesProxy, SQLiteRunsProxy
from .utils import transform_frame

class SQLiteProxy(DatabaseProxy):
    """Factory for SQLite database proxy interfaces.

    It declares the set of connections to objects that any database proxy must implement, including:
    - PointClouds
    - Positions
    - Cones
    - NoCones
    - Runs
    """

    def __init__(self, db_path: str):
        """Initialize the SQLite database proxy.

        Args:
            db_path (str): The path to the SQLite database.
        """
        self._db_path = db_path

    def get_point_clouds(self) -> AbstractPointCloudsProxy:
        """Return the point clouds proxy."""
        return SQLitePointCloudsProxy(self._db_path)

    def get_positions(self) -> AbstractPositionsProxy:
        """Return the positions proxy."""
        return SQLitePositionsProxy(self._db_path)

    def get_cones(self) -> AbstractConesProxy:
        """Return the cones proxy."""
        return SQLiteConesProxy(self._db_path)

    def get_no_cones(self) -> AbstractNoConesProxy:
        """Return the no cones proxy."""
        return SQLiteNoConesProxy(self._db_path)

    def get_runs(self) -> AbstractRunsProxy:
        """Return the runs proxy."""
        return SQLiteRunsProxy(self._db_path)

    def get_lidar_frames(self, run: str, step: int = 100) -> Tuple[list, list]:
        """Return the lidar frames from the database."""
        frames = []
        cones = []

        positions = self.get_positions.read(run_name=run, projection=['pos_x', 'pos_y', 'pos_z', 'ori_x', 'ori_y', 'ori_z', 'ori_w', 'timestamp'])
        for position in positions[::step]:
            position_datetime = datetime.fromtimestamp(position[7])
            frame_lidar = self.get_point_clouds.read(run_name=run, datetime=position_datetime, projection=['x', 'y', 'z'], before=position_datetime, delta=100)
            position_cones = self.get_cones.read(run_name=run, datetime=position_datetime, projection=['x', 'y', 'z'], visible_check=True, before=position_datetime, delta=100)

            # https://automaticaddison.com/how-to-convert-a-quaternion-to-a-rotation-matrix/
            frame_lidar, position_cones = transform_frame(frame_lidar, position_cones, position)
            frames.append(frame_lidar)
            cones.append(position_cones)

        return frame_lidar, cones