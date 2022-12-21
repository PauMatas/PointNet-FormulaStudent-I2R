from ..base_proxy import DatabaseProxy, AbstractBaseProxy, AbstractPointCloudsProxy, AbstractPositionsProxy, AbstractConesProxy, AbstractNoConesProxy, AbstractRunsProxy
from .proxys import SQLitePointCloudsProxy, SQLitePositionsProxy, SQLiteConesProxy, SQLiteNoConesProxy, SQLiteRunsProxy

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

    def get_lidar_frames(self):
        """Return the lidar frames from the database."""


        # positions = SQLitePositionsProxy().read(run_name=run, projection=['x', 'y', 'z'])
        # for position in positions[::100]:
        #     position_datetime = datetime.fromtimestamp(position[3])
        #     frame_lidar = SQLitePointCloudsProxy().read(run_name=run, datetime=position_datetime, projection=['x', 'y', 'z'])
        #     cons = SQLiteConesProxy().read(run_name=run, datetime=position_datetime, projection=['x', 'y', 'z'])
        raise NotImplementedError