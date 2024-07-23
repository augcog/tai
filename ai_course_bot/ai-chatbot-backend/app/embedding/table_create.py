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

# Load data from the pickle file
with open('ai_course_bot/ai-chatbot-backend/app/embedding/recursive_seperate_none_BGE_embedding_400_106_full.pkl', 'rb') as f:
    data_to_store = pickle.load(f)



# Define the insert function

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

db = connect('embeddings.db')
cur = db.cursor()
cur.execute('Drop table IF EXISTS embeddings;')
cur.execute('create virtual table embeddings using vss0(embedding(1024) factory="Flat,IDMap2" metric_type=INNER_PRODUCT);')
embedding_list = data_to_store['embedding_list']
denses = [embedding['dense_vecs'].tolist() for embedding in embedding_list]
insert(cur, denses)
db.commit()
# cur.execute("""
#     select rowid, distance
#     from embeddings
#     where vss_search(
#         embedding,
#         (select embedding from embeddings where rowid = 1)
#     )
#     limit 10;
# """)
# Verify the number of rows inserted
cur.execute("SELECT COUNT(*) AS row_count FROM embeddings")
row_count = cur.fetchone()

print(f"Number of rows inserted: {row_count['row_count']}")
# cur.execute("SELECT * FROM embeddings")
rows = cur.fetchall()
for i in rows:
    print(i)
# Perform a VSS search (example)
try:
    cur.execute("""
        SELECT 
            rowid, 
            distance
        FROM embeddings
        WHERE vss_search(
            embedding,
            (SELECT embedding FROM embeddings WHERE rowid = 1000)
        )
        LIMIT 5;
    """)
    results = cur.fetchall()
    for result in results:
        print(f"rowid: {result['rowid']}, distance: {result['distance']}")
except sqlite3.OperationalError as e:
    print(f"Query operation failed: {e}")
db.commit()
cur.execute("VACUUM;")
db.close()
