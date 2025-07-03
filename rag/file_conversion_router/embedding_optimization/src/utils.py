import logging
import pickle
from typing import List, Optional

from transformers import AutoModelForCausalLM, AutoTokenizer

from file_conversion_router.classes.chunk import Chunk

logger = logging.getLogger(__name__)


def load_pkl(file_path: str) -> List[Chunk]:
    with open(file_path, "rb") as f:
        return pickle.load(f)


def save_pkl(chunks: List[Chunk], file_path: str):
    with open(file_path, "wb") as f:
        pickle.dump(chunks, f)


def ensure_model_downloaded(
    model_name: str, model_cache_dir: Optional[str] = None
) -> str:
    """
    Ensure that the specified model is downloaded and available locally.

    Args:
        model_name (str): Identifier for the model (e.g., 'meta-llama/Meta-Llama-3-8B').
        model_cache_dir (Optional[str]): Directory to store the model. If None, uses Hugging Face's default cache.

    Returns:
        str: Path to the downloaded model directory.

    Raises:
        RuntimeError: If the model cannot be downloaded or loaded.
    """
    try:
        logger.info(f"Checking availability of model '{model_name}'.")
        # Attempt to load the tokenizer and model; this will download them if not present
        tokenizer = AutoTokenizer.from_pretrained(model_name, cache_dir=model_cache_dir)
        model = AutoModelForCausalLM.from_pretrained(
            model_name, cache_dir=model_cache_dir
        )
        model_path = model.name_or_path
        logger.info(f"Model '{model_name}' is available at '{model_path}'.")
        return model_path
    except Exception as e:
        logger.error(f"Failed to download or load model '{model_name}': {e}")
        raise RuntimeError(f"Failed to download or load model '{model_name}': {e}")
