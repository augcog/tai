import sqlite3
import pickle
import json
import numpy as np
import os

# Paths to your vector and vss extensions
EXT_VECTOR_PATH = "rag/file_conversion_router/embedding/dist/debug/vector0"
EXT_VSS_PATH = "rag/file_conversion_router/embedding/dist/debug/vss0"
BGE = True

# Modify this path to the directory containing the embedding pickle files and the database
DIRECTORY_PATH = "roarai/rag/file_conversion_router/embedding"


# Connect to the SQLite database and load extensions
def connect(path=":memory:"):
    db = sqlite3.connect(path)
    db.enable_load_extension(True)
    db.execute(
        "create temp table base_functions as select name from pragma_function_list"
    )
    db.execute("create temp table base_modules as select name from pragma_module_list")
    db.load_extension(EXT_VECTOR_PATH)
    db.execute(
        "create temp table vector_loaded_functions as select name from pragma_function_list where name not in (select name from base_functions) order by name"
    )
    db.execute(
        "create temp table vector_loaded_modules as select name from pragma_module_list where name not in (select name from base_modules) order by name"
    )
    db.execute("drop table base_functions")
    db.execute("drop table base_modules")
    db.execute(
        "create temp table base_functions as select name from pragma_function_list"
    )
    db.execute("create temp table base_modules as select name from pragma_module_list")
    db.load_extension(EXT_VSS_PATH)
    db.execute(
        "create temp table vss_loaded_functions as select name from pragma_function_list where name not in (select name from base_functions) order by name"
    )
    db.execute(
        "create temp table vss_loaded_modules as select name from pragma_module_list where name not in (select name from base_modules) order by name"
    )
    db.row_factory = sqlite3.Row
    return db


def execute_all(cursor, sql, args=None):
    if args is None:
        args = []
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
        """,
        [json.dumps(data_list)],
    )


def get_columns(pickle_data):
    columns = []
    keys = list(pickle_data.keys())

    for key in keys:
        if len(pickle_data[key]) > 0 and isinstance(pickle_data[key][0], dict):
            columns.extend(
                [f"{key}_{sub_key}" for sub_key in pickle_data[key][0].keys()]
            )
        else:
            columns.append(key)
    print("Columns:", columns)

    return columns, keys


def get_structure_debug(pickle_data):
    for key, value in pickle_data.items():
        print(f"Key: {key}")
        print(f"Type: {type(value)}")
        if isinstance(value, np.ndarray):
            print(f"Length: {len(value)}")
            if len(value) > 0:
                print(f"Sample Value Type: {type(value[0])}")
                if isinstance(value[0], dict):
                    print(f"Sample Value Keys: {list(value[0].keys())}")
        elif isinstance(value, dict):
            print(f"Keys: {list(value.keys())}")


def create_embedding_table(pickle_data):
    os.makedirs(DIRECTORY_PATH, exist_ok=True)
    db_path = os.path.join(DIRECTORY_PATH, "embeddings.db")
    db = connect(db_path)
    print(db_path)
    cur = db.cursor()
    cur.execute("Drop table IF EXISTS embeddings;")
    cur.execute(
        """
                create virtual table embeddings using vss0(
                embedding(1024)
                factory="Flat,IDMap2" metric_type=INNER_PRODUCT);
                """
    )

    embedding_list = pickle_data["embedding_list"]
    denses = [embedding["dense_vecs"].tolist() for embedding in embedding_list]
    insert(cur, denses)
    db.commit()

    # Verify the number of rows inserted
    cur.execute("SELECT COUNT(*) AS row_count FROM embeddings")
    row_count = cur.fetchone()
    print(f"Number of rows inserted: {row_count['row_count']}")
    rows = cur.fetchall()
    for i in rows:
        print(i)

    # Perform a VSS search (example)
    try:
        cur.execute(
            """
            SELECT 
                rowid, 
                distance
            FROM embeddings
            WHERE vss_search(
                embedding,
                (SELECT embedding FROM embeddings WHERE rowid = 1000)
            )
            LIMIT 5;
        """
        )
        results = cur.fetchall()
        for result in results:
            print(f"rowid: {result['rowid']}, distance: {result['distance']}")
    except sqlite3.OperationalError as e:
        print(f"Query operation failed: {e}")
    db.commit()
    cur.execute("VACUUM;")
    db.close()

    return cur


def create_main_table(filename, pickle_data):
    if filename.endswith(".pkl"):
        table_name = os.path.splitext(os.path.basename(filename))[0]
        print(table_name)
        database_name = table_name + ".db"
    else:
        raise ValueError("The provided file does not have a .pkl extension")

    os.makedirs(DIRECTORY_PATH, exist_ok=True)
    db_path = os.path.join(DIRECTORY_PATH, database_name)
    print(db_path)
    db = sqlite3.connect(db_path)
    cur = db.cursor()

    columns, keys = get_columns(pickle_data)

    cur.execute(f"Drop table IF EXISTS {table_name};")
    column_definitions = "rowid INTEGER PRIMARY KEY AUTOINCREMENT, " + ", ".join(
        [f"{col} TEXT" for col in columns]
    )
    cur.execute(f"CREATE TABLE {table_name} ({column_definitions})")

    cur.execute(f"PRAGMA table_info({table_name})")
    rows = cur.fetchall()
    column_names = [row[1] for row in rows]

    for i in range(len(pickle_data[columns[0]])):
        row = []
        for col in columns:
            # Handle embedding_list specially
            if col.startswith("embedding_list"):
                embedding_key = col.replace("embedding_list_", "")
                row.append(str(pickle_data["embedding_list"][i][embedding_key]))
            else:
                row.append(str(pickle_data[col][i]))

        placeholders = ", ".join(["?" for _ in columns])
        cur.execute(
            f"""
            INSERT INTO {table_name} ({', '.join(columns)}) 
            VALUES ({placeholders})
        """,
            row,
        )
    db.commit()

    cur.execute(f"SELECT * FROM {table_name} LIMIT 3")
    rows = cur.fetchall()
    for row in rows:
        print(row)

    # Commit and close the connection
    db.close()


def main():
    ee106b = "rag/file_conversion_router/embedding/eecs106b.pkl"
    path_to_pickle = ee106b
    # path_to_pickle = "rag/file_conversion_router/embedding/cs61a.pkl"

    with open(path_to_pickle, "rb") as f:
        data_loaded = pickle.load(f)

    create_embedding_table(data_loaded)
    create_main_table(path_to_pickle, data_loaded)


if __name__ == "__main__":
    main()
