import sqlite3
import pickle
import json
import numpy as np
import random
import os

# Paths to your vector and vss extensions
EXT_VECTOR_PATH = "../embedding/dist/debug/vector0.dylib"
EXT_VSS_PATH = "../embedding/dist/debug/vss0.dylib"
BGE = True

# Connect to the SQLite database and load extensions
def connect(path=":memory:"):
    db = sqlite3.connect(path)
    db.enable_load_extension(True)
    db.execute("create temp table base_functions as select name from pragma_function_list")
    db.execute("create temp table base_modules as select name from pragma_module_list")
    db.load_extension(EXT_VECTOR_PATH)
    db.execute("create temp table vector_loaded_functions as select name from pragma_function_list where name not in (select name from base_functions) order by name")
    db.execute("create temp table vector_loaded_modules as select name from pragma_module_list where name not in (select name from base_modules) order by name")
    db.execute("drop table base_functions")
    db.execute("drop table base_modules")
    db.execute("create temp table base_functions as select name from pragma_function_list")
    db.execute("create temp table base_modules as select name from pragma_module_list")
    db.load_extension(EXT_VSS_PATH)
    db.execute("create temp table vss_loaded_functions as select name from pragma_function_list where name not in (select name from base_functions) order by name")
    db.execute("create temp table vss_loaded_modules as select name from pragma_module_list where name not in (select name from base_modules) order by name")
    db.row_factory = sqlite3.Row
    return db

def execute_all(cursor, sql, args=None):
    if args is None: args = []
    results = cursor.execute(sql, args).fetchall()
    return list(map(lambda x: dict(x), results))

def insert(cur, data_list):
    execute_all(
        cur,
        """
        insert into embeddings(rowid, embedding)
            select
                key, 
                value
            from json_each(?);
        """, [json.dumps(data_list)])

def create_table(filename, pickle_data):
    if filename.endswith('.pkl'):
        sql_filename = os.path.splitext(filename)[0] + '.db'
    else:
        raise ValueError("The provided file does not have a .pkl extension")
    db = connect(sql_filename)
    cur = db.cursor()
    cur.execute('Drop table IF EXISTS embeddings;')
    cur.execute('create virtual table embeddings using vss0(embedding(1024) factory="Flat,IDMap2" metric_type=INNER_PRODUCT);')
    embedding_list = pickle_data['embedding_list']
    denses = [embedding['dense_vecs'].tolist() for embedding in embedding_list]
    insert(cur, denses)
    db.commit()
    return cur