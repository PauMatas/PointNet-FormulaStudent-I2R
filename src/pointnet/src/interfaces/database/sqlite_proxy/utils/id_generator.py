from typing import List, Tuple, Union
import uuid

from .querys import execute_query

def generate_and_lookup_id(db_path: str, table_name: str, new_ids: list) -> Tuple[str, bool]:
    id_ = str(uuid.uuid4()).replace('-','')
    existing_ids = execute_query(db_path=db_path, query=f'SELECT * FROM {table_name} WHERE id = \'{id_}\'', fetch_result=True)
    return id_, (len(existing_ids) == 0 and id_ not in new_ids)

def new_id(db_path: str, table_name: str, n: int = 1) -> Union[str, List[str]]:
    """Generate a new unique id"""
    ids = []
    for _ in range(n):
        id_, is_unique = generate_and_lookup_id(db_path=db_path, table_name=table_name, new_ids=ids)
        while not is_unique:
            id_, is_unique = generate_and_lookup_id(db_path=db_path, table_name=table_name, new_ids=ids)
        ids.append(id_)
    return ids[0] if n == 1 else ids
