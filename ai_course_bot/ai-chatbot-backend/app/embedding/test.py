import sqlite3
import pickle
import json
import numpy as np

# Paths to your vector and vss extensions
EXT_VECTOR_PATH = "./dist/debug/vector0"
EXT_VSS_PATH = "./dist/debug/vss0"
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

# Load data from the pickle file
with open('recursive_seperate_none_BGE_embedding_400_106_full.pkl', 'rb') as f:
    data_to_store = pickle.load(f)

# Function to convert the dictionary to a JSON array
def dict_to_json_array(data_to_store):
    # Convert numpy arrays to lists
    id_list = data_to_store['id_list'].tolist()
    doc_list = data_to_store['doc_list'].tolist()
    embedding_list = data_to_store['embedding_list']
    if BGE:
        embedding_list = [embedding['dense_vecs'] for embedding in embedding_list]
    embedding_list = [embedding.tolist() for embedding in embedding_list]
    url_list = data_to_store['url_list'].tolist()
    time_list = data_to_store['time_list'].tolist()

    # Create a list of dictionaries
    json_array = []
    for i in range(len(id_list)):
        json_array.append({
            'id': id_list[i],
            'doc': doc_list[i],
            'embedding': embedding_list[i],
            'URL': url_list[i],
            'time': time_list[i]
        })

    return json_array

# Convert the dictionary to a JSON array
json_array = dict_to_json_array(data_to_store)

# Connect to SQLite database
conn = connect('embeddings.db')

# Create table with VSS vector type


# Function to insert embeddings
def insert_embedding(conn, record):
    # print(len(record['embedding']))
    json_array = json.dumps(record['embedding'])
    conn.execute('''
        INSERT OR REPLACE INTO embeddings (id, doc, embedding, URL, time)
        VALUES (?, ?, ?, ?, ?)
    ''', (record['id'], record['doc'], json_array, record['URL'], record['time']))

# Insert data into the table
for record in json_array:
    # Ensure embedding is a list
    if isinstance(record['embedding'], np.ndarray):
        record['embedding'] = record['embedding'].tolist()
    insert_embedding(conn, record)

print(json_array[100]['id'])


print("Data has been successfully inserted into the SQLite database.")


# Function to perform a vector similarity search
def search_embeddings(conn, query_embedding, top_k=5):
    query_embedding_json = json.dumps(query_embedding)
    cursor = conn.execute('''
        SELECT id, doc, URL, time where vss_search(embedding, vss_search_params(json(?), ?)) AS similarity
        FROM embeddings
        ORDER BY similarity DESC
        LIMIT ?
    ''', (query_embedding_json, top_k))

    # Fetch results
    results = cursor.fetchall()

    # Print column names
    column_names = [description[0] for description in cursor.description]
    print("Attributes in results:", column_names)

    return results


results = search_embeddings(conn, json_array[100]['embedding'], top_k=5)
# Print results
for result in results:
    print(f"ID: {result['id']}, Similarity: {result['similarity']}, URL: {result['URL']}, Time: {result['time']}")


print()

conn.close()









