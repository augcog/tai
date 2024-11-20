import pickle
import os


def save_persistent_cache(data, cache_path):
    if not os.path.exists(os.path.dirname(cache_path)):
        os.makedirs(os.path.dirname(cache_path))
    with open(cache_path, "wb") as f:
        pickle.dump(data, f)


def load_persistent_cache(cache_path):
    if os.path.exists(cache_path):
        with open(cache_path, "rb") as f:
            return pickle.load(f)
    else:
        return {}


def dummy_convert(in_path, out_path):
    with open(out_path, "a") as f:
        f.write(in_path.stem + "\n")


