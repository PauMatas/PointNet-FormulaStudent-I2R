# Create a sqlite3 database with the following tables:
# - point_clouds
# - positions
# - cones
# - no_cones
# - runs

from os import path
import sqlite3 as sql
import sys

sys.path.append(path.join(path.dirname(__file__), '../../../..'))
from src.interfaces.database.sqlite_proxy.proxys import SQLitePointCloudsProxy, SQLitePositionsProxy, SQLiteConesProxy, SQLiteNoConesProxy, SQLiteRunsProxy


DB_PATH = path.join(path.dirname(path.abspath(__file__)), '../../../../data/database.sqlite3')

conn = sql.connect(DB_PATH)
cursor = conn.cursor()

for proxy in [SQLitePointCloudsProxy, SQLitePositionsProxy, SQLiteConesProxy, SQLiteNoConesProxy, SQLiteRunsProxy]:
    table = proxy(DB_PATH)
    query = f'CREATE TABLE IF NOT EXISTS {table._table_name} ('
    query += ','.join([column.create_query for column in table._columns])
    for param in table._creation_params:
        query += f',{param}'
    query += ')'
    cursor.execute(query)
    
conn.commit()
conn.close()