from abc import ABC, abstractmethod

from .proxys import AbstractPointCloudsProxy, AbstractPositionsProxy, AbstractConesProxy, AbstractNoConesProxy, AbstractRunsProxy

class DatabaseProxy(ABC):
    """Abstract factory for database proxy interfaces.

    It declares the set of connections to objects that any database proxy must implement, including:
    - PointClouds
    - Positions
    - Cones
    - NoCones
    - Runs
    """

    @abstractmethod
    def get_point_clouds(self) -> AbstractPointCloudsProxy:
        """Return the point clouds proxy."""
        raise NotImplementedError

    @abstractmethod
    def get_positions(self) -> AbstractPositionsProxy:
        """Return the positions proxy."""
        raise NotImplementedError

    @abstractmethod
    def get_cones(self) -> AbstractConesProxy:
        """Return the cones proxy."""
        raise NotImplementedError

    @abstractmethod
    def get_no_cones(self) -> AbstractNoConesProxy:
        """Return the no cones proxy."""
        raise NotImplementedError

    @abstractmethod
    def get_runs(self) -> AbstractRunsProxy:
        """Return the runs proxy."""
        raise NotImplementedError

    def get_lidar_frames(self):
        """Return the lidar frames from a database."""
        raise NotImplementedError
