from itertools import groupby
from typing import List

from .querys import execute_query
from .column import Column, NUMERIC_TYPES


def insert_row(data_base_path: str, table_name: str, columns: List[Column], **kwargs):
    """Insert a row into the table"""

    values = [
        str(kwargs[column.name])
        if column.type in NUMERIC_TYPES
        else f"'{str(kwargs[column.name])}'"
        for column in columns
    ]
    if not values:
        raise ValueError('No values to insert in {table_name}. Please, provide at least one value.')
    values = ', '.join(values)

    query = f"INSERT INTO {table_name} VALUES({values})"
    execute_query(db_path=data_base_path, query=query)

def insert_rows(data_base_path: str, table_name: str, columns: List[Column], rows: List[tuple]):
    """Insert multiple rows into the table"""

    question_marks = ', '.join(['?' for _ in range(len(columns))])
    query = f"INSERT INTO {table_name} VALUES({question_marks})"
    execute_query(db_path=data_base_path, query=query, rows=rows)

def contains_one_run(rows: List[tuple], columns: List[Column]) -> bool:
    """Return True if all the rows belong to only one run, False otherwise"""
    # -1 because the first column is the id
    run_column_index = next((index for index, column in enumerate(columns) if column.name == 'run_name')) - 1
    grouped_rows = groupby(rows, lambda row: row[run_column_index])
    return next(grouped_rows, True) and not next(grouped_rows, False)