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
from src.interfaces.database.sqlite_proxy.factory import SQLiteProxy


DB_PATH = path.join(path.dirname(path.abspath(__file__)), '../../../../data/database.sqlite3')

SQLiteProxy(DB_PATH).get_no_cones().clean()