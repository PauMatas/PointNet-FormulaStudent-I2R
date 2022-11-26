import sqlite3 as sql

from tables import TABLES, DATA_BASE_PATH

if __name__ == '__main__':
    for table_class in TABLES:
        table = table_class()
        try:
            main_conn = sql.connect(DATA_BASE_PATH)
            main_cursor = main_conn.cursor()
            main_cursor.execute(f"SELECT * FROM {table.name}")
            main_conn.close()
        except sql.OperationalError:
            table.create_table()