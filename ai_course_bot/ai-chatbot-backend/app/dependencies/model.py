import torch
import transformers


def get_model_pipeline():
    """
    Loads and returns the text-generation pipeline.
    This is the heavy part that we want to override in tests.
    """
    model_id = "meta-llama/Meta-Llama-3-8B-Instruct"
    pipeline = transformers.pipeline(
        "text-generation",
        model=model_id,
        model_kwargs={"torch_dtype": torch.bfloat16},
        device=-1  # force CPU by default
    )
    return pipeline
