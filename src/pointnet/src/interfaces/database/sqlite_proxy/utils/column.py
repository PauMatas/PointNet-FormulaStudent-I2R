NUMERIC_TYPES = ['INTEGER', 'REAL', 'NUMERIC', 'DOUBLE', 'FLOAT', 'DECIMAL', 'VARCHAR']

class Column:
    """Column of a table"""

    def __init__(self, name: str, col_type: str):
        """Initialize the column

        Args:
            name (str): The name of the column
            type (str): The type of the column
        """

        self.name = name
        self.type = col_type.upper()
        self.create_query = f"{self.name} {self.type}"

    def __str__(self):
        """String representation of the column"""
    
        return f"Column({self.name}, {self.type})"

    def __repr__(self):
        """Representation of the column"""

        return self.__str__()

    def __contains__(self, item: str):
        """Check if the column contains the item

        Args:
            item (str): The item to check

        Returns:
            bool: True if the item is in the column, False otherwise
        """

        return item == self.name