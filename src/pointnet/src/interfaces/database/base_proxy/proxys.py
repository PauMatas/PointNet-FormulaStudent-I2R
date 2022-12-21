from abc import ABC, abstractmethod
from typing import List, Union

from src.objects import BoundingBox

class AbstractBaseProxy(ABC):
    """Abstract class for proxy interfaces.

    It declares the set of operations that any proxy must implement, including:
    - Insert
    - Read
    - Update
    - Delete
    """

    @abstractmethod
    def insert(self, **kwargs):
        """Insert objects in the database."""
        raise NotImplementedError
    
    @abstractmethod
    def read(self, **kwargs) -> List[tuple]:
        """Read objects from the database."""
        raise NotImplementedError

    @abstractmethod
    def update(self, **kwargs):
        """Update objects in the database."""
        raise NotImplementedError

    @abstractmethod
    def delete(self, **kwargs):
        """Delete objects from the database."""
        raise NotImplementedError


class AbstractPointCloudsProxy(AbstractBaseProxy):
    """Abstract class for point clouds proxy interfaces.

    It declares the set of operations that any point clouds proxy must implement, including:
    - Insert
    - Read
    - Update
    - Delete
    - Extract Bounding Boxes
    """

    @abstractmethod
    def extract_bounding_boxes(self, run: str = None, only_sizes: bool = False, **kwargs) -> List[Union[BoundingBox, int]]:
        """Extract bounding boxes from point clouds."""
        raise NotImplementedError


class AbstractPositionsProxy(AbstractBaseProxy):
    """Abstract class for positions proxy interfaces.

    It declares the set of operations that any positions proxy must implement, including:
    - Insert
    - Read
    - Update
    - Delete
    """
    pass


class AbstractConesProxy(AbstractBaseProxy):
    """Abstract class for cones proxy interfaces.

    It declares the set of operations that any cones proxy must implement, including:
    - Insert
    - Read
    - Update
    - Delete
    """
    pass


class AbstractNoConesProxy(AbstractBaseProxy):
    """Abstract class for no cones proxy interfaces.

    It declares the set of operations that any no cones proxy must implement, including:
    - Insert
    - Read
    - Update
    - Delete
    """
    pass


class AbstractRunsProxy(AbstractBaseProxy):
    """Abstract class for runs proxy interfaces.

    It declares the set of operations that any runs proxy must implement, including:
    - Insert
    - Read
    - Update
    - Delete
    """
    pass
