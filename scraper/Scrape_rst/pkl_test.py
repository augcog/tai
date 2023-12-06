import pickle

def load_data_from_path(path):
    """
    Load data from a pickle file at the specified path.

    Parameters:
    - path: Full path of the pickle file to load data from.

    Returns:
    - Loaded data from the pickle file.
    """
    with open(path, 'rb') as file:
        loaded_data = pickle.load(file)
    return loaded_data

# Example of using the function:

data = load_data_from_path('Sawyer/index.pkl')
for i in data:
    print(i['Page_table'])
    print(i['Page_path'])
    print(i['Segment_print'])
    print("=====================================")
