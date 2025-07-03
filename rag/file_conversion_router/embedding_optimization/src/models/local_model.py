import logging
from typing import Optional

import torch
from transformers import AutoModelForCausalLM, AutoTokenizer
from transformers import logging as hf_logging
from transformers import pipeline

from file_conversion_router.embedding_optimization.src.models.base_model import (
    BaseModel,
)
from file_conversion_router.embedding_optimization.src.utils import (
    ensure_model_downloaded,
)

# Suppress excessive logging from transformers library
hf_logging.set_verbosity_error()

logger = logging.getLogger(__name__)


class LocalLLama3Model(BaseModel):
    def __init__(
        self,
        model_name: str,
        model_cache_dir: Optional[str] = None,
        device: Optional[int] = None,
        max_length: int = 256,
        num_beams: int = 4,
        **kwargs,
    ):
        """
        Initialize the LocalLLama3Model with the given model name and configurations.

        Args:
            model_name (str): Identifier for the Llama 3 model (e.g., 'meta-llama/Meta-Llama-3-8B').
            model_cache_dir (Optional[str]): Directory to store the downloaded model. Defaults to None, which uses Hugging Face's default cache.
            device (Optional[int]): Device ID for CUDA (e.g., 0 for GPU). If None, uses GPU if available, else CPU.
            max_length (int): Maximum length of the generated summary.
            num_beams (int): Number of beams for beam search (controls diversity).
            **kwargs: Additional keyword arguments for the pipeline.
        """
        self.model_name = model_name
        self.model_dir = model_cache_dir
        self.device = (
            device if device is not None else (0 if torch.cuda.is_available() else -1)
        )
        self.max_length = max_length
        self.num_beams = num_beams
        self.pipeline_kwargs = kwargs
        self.model = None
        self.tokenizer = None
        self.summarizer = None
        self._is_loading = False  # Flag to prevent concurrent loading
        # self.load_model()

    def _load_model_if_needed(self):
        """
        Load the model and tokenizer if they haven't been loaded yet.
        This method ensures that the model is loaded only once, even in multi-threaded environments.
        """
        if (
            self.model is not None
            and self.tokenizer is not None
            and self.summarizer is not None
        ):
            return  # Model is already loaded

        if self._is_loading:
            logger.info("Model is currently being loaded by another process.")
            # Optionally, implement waiting or raise an exception
            raise RuntimeError(
                "Model is currently being loaded. Please try again shortly."
            )

        self._is_loading = True
        try:
            logger.info(f"Initializing LocalLLama3Model for '{self.model_name}'.")

            model_path = ensure_model_downloaded(self.model_name, self.model_dir)
            logger.info(f"Model '{self.model_name}' is available at '{model_path}'.")

            logger.info("Loading tokenizer...")
            self.tokenizer = AutoTokenizer.from_pretrained(model_path, use_fast=True)
            logger.info("Tokenizer loaded successfully.")

            logger.info("Loading model...")
            self.model = AutoModelForCausalLM.from_pretrained(model_path)
            if self.device >= 0:
                self.model.to(self.device)
                logger.info(f"Model moved to device {self.device}.")
            else:
                logger.info("Model is running on CPU.")
            logger.info("Model loaded successfully.")

            logger.info("Initializing summarization pipeline...")
            # Remove any unexpected kwargs
            sanitized_kwargs = {
                k: v for k, v in self.pipeline_kwargs.items() if k != "model_path"
            }
            self.summarizer = pipeline(
                "text-generation",
                model=self.model,
                tokenizer=self.tokenizer,
                device=self.device if self.device >= 0 else -1,
                **sanitized_kwargs,
            )
            logger.info("Summarization pipeline initialized successfully.")
        except Exception as e:
            logger.error(f"Failed to load LocalLLama3Model: {e}")
            raise RuntimeError(f"Failed to load LocalLLama3Model: {e}")
        finally:
            self._is_loading = False

    def generate_summary(self, text: str) -> str:
        if not text.strip():
            logger.warning("Empty text provided for summarization.")
            raise ValueError("Input text is empty.")

        # Ensure the model is loaded (lazy loading)
        try:
            self._load_model_if_needed()
        except RuntimeError as e:
            logger.error(e)
            raise

        try:
            logger.debug(f"Generating summary for text: {text[:60]}...")
            prompt = (
                "Please provide a concise summary of the following text while retaining all key information.\n"
                f"Text: {text}\n"
                "Summary:"
            )

            # Generate summary with refined parameters
            summary_list = self.summarizer(
                prompt,
                max_length=self.max_length,
                min_length=int(
                    self.max_length * 0.5
                ),  # Optional: Enforce minimum length
                num_beams=self.num_beams,
                early_stopping=True,
                no_repeat_ngram_size=3,
                temperature=0.7,
                top_p=0.9,
                repetition_penalty=1.2,
                length_penalty=2.5,  # Increase to penalize longer outputs
                do_sample=False,  # Deterministic generation for concise output
            )

            # Extract the summary from the generated text
            generated_text = summary_list[0]["generated_text"]
            # Adjust extraction based on the new prompt
            summary = generated_text.split("Summary:")[-1].strip()
            logger.debug(f"Generated summary: {summary}")

            return summary
        except Exception as e:
            logger.error(f"Error during summarization: {e}")
            raise ValueError(f"Failed to generate summary: {e}")
