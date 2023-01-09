from typing import List, Union, Any
import sqlite3 as sql

from .column import Column, NUMERIC_TYPES

def execute_query(query: str, db_path: str, rows: list = None, fetch_result: bool = False) -> Union[List[tuple], None]:
    """Execute a query"""
    conn = sql.connect(db_path)
    cursor = conn.cursor()

    if rows is None:
        cursor.execute(query)
    elif rows:
        cursor.executemany(query, rows)
    else:
        raise ValueError('Invalid rows argument')
    
    if fetch_result:
        result = cursor.fetchall()
        conn.close()
        return result
    
    conn.commit()
    conn.close()


def parse_query_filters(kwargs: dict, columns: List[Column]) -> str:
    """Parses the filters to be used in a query or returns an empty string if no filters are found"""
    conditions = [
        f"{column.name} = {str(kwargs[column.name])}"
        if column.type in NUMERIC_TYPES
        else f"{column.name} = '{str(kwargs[column.name])}'"
        for column in columns
        if column.name in kwargs
    ]
    return ' AND '.join(conditions)


def parse_additional_query_arguments(arg: str, kwargs: dict, not_found: Any = None, sep: str = ',') -> Union[str, Any]:
    """Parses additional arguments as a string separated with sep to be used in a query or returns None if not found"""
    if arg in kwargs:
        if not isinstance(kwargs[arg], list):
            kwargs[arg] = [kwargs[arg], ]
        return sep.join(list(map(str, kwargs[arg])))
    return not_found
