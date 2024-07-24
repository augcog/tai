import sqlite3
import pickle
import json
import numpy as np
import random

# Paths to your vector and vss extensions
EXT_VECTOR_PATH = "ai_course_bot/ai-chatbot-backend/app/embedding/dist/debug/vector0"
EXT_VSS_PATH = "ai_course_bot/ai-chatbot-backend/app/embedding/dist/debug/vss0"
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