from functools import lru_cache
from typing import List, Dict, Optional
import torch
import transformers
from abc import ABC, abstractmethod
from openai import OpenAI
import os

from rag.file_organizer.src.utils.logging_service import get_logger
from rag.file_organizer.src.config.config import load_profiles

logger = get_logger(__name__)


class LLMBase(ABC):
    """Base class for models, providing a common interface."""

    @abstractmethod
    async def chat(self, messages: List[Dict[str, str]]) -> str:
        """Generate text based on the provided messages."""
        raise NotImplementedError("Subclasses should implement this method.")


class MockModel(LLMBase):
    """Mock model for testing purposes."""

    async def chat(self, messages: List[Dict[str, str]]) -> str:
        """Generate mock text for testing."""
        return "{'file': 'This is a mock response for testing purposes.'}"


class OpenAIModel(LLMBase):
    def __init__(
        self,
        model_name: str,
        api_key=None,
        max_new_tokens=1024,
        temperature: float = 0.7,
    ):
        self.api_key = api_key or os.getenv("OPENAI_API_KEY") or ""
        if not self.api_key:
            raise ValueError(
                "OpenAI API key must be provided either as an argument or via environment variable."
            )
        self.model = model_name
        self.client = OpenAI(api_key=self.api_key)
        self.max_new_tokens = max_new_tokens
        self.temperature = temperature

    async def chat(
        self,
        messages: List[Dict[str, str]],
        max_new_tokens: Optional[int] = None,
        temperature: Optional[float] = None,
    ) -> str:
        try:
            resp = self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                stream=False,
                temperature=temperature or self.temperature,
                max_tokens=max_new_tokens or self.max_new_tokens,
                top_p=1,
            )
            return resp.choices[0].message.content
        except Exception as e:
            logger.error(f"OpenAIModel chat error: {e}")
            raise


class LocalHFModel(LLMBase):
    """
    provider:  local_hf
    model_id:  meta-llama/Meta-Llama-3-8B-Instruct
    extra:     {quantization: "4bit" | "8bit" | "none"}
    """

    def __init__(
        self,
        model_id: str,
        quantization: str = "none",
        max_new_tokens: int = 1024,
        temperature: float = 0.7,
        device: str = "auto",
    ):
        self.model_id = model_id
        self.quantization = quantization
        self.max_new_tokens = max_new_tokens
        self.temperature = temperature
        self.device = self._resolve_device(device)
        self._init_pipeline()

    # ------------------------ public API ------------------------
    async def chat(  # sync is fine, just wrap it in anyio.to_thread
        self,
        messages: List[Dict[str, str]],
        max_new_tokens: Optional[int] = None,
        temperature: Optional[float] = None,
    ) -> str:
        prompt = self._build_prompt(messages)
        result = self.pipeline(
            prompt,
            max_new_tokens=max_new_tokens or self.max_new_tokens,
            temperature=temperature or self.temperature,
            do_sample=True,
            return_full_text=False,
        )[0]["generated_text"]

        return self._strip_special_tokens(result)

    # ------------------------ private methods ------------------------
    def _resolve_device(self, device: str) -> str:
        if device != "auto":
            return device
        if torch.cuda.is_available():
            return "cuda"
        elif hasattr(torch, "mps") and torch.backends.mps.is_available():
            return "mps"
        else:
            return "cpu"

    def _init_pipeline(self):
        dtype = torch.float32 if self.device == "cpu" else torch.bfloat16
        model_kw = {"torch_dtype": dtype}

        if self.quantization == "4bit":
            model_kw |= {
                "load_in_4bit": True,
                "bnb_4bit_compute_dtype": torch.float16,
                "bnb_4bit_quant_type": "nf4",
                "bnb_4bit_use_double_quant": True,
            }
        elif self.quantization == "8bit":
            model_kw["load_in_8bit"] = True

        logger.info(f"Loading {self.model_id} ({self.quantization}) on {self.device}")
        tok = transformers.AutoTokenizer.from_pretrained(self.model_id)
        self.pipeline = transformers.pipeline(
            "text-generation",
            model=self.model_id,
            tokenizer=tok,
            device=0 if self.device == "cuda" else -1,
            model_kwargs=model_kw,
        )

    # ---------------------------prompt helpers ------------------------------
    def _build_prompt(self, msgs: List[Dict[str, str]]) -> str:
        try:
            tok = self.pipeline.tokenizer
            return tok.apply_chat_template(
                msgs, tokenize=False, add_generation_prompt=True
            )
        except Exception:
            return self._fallback_prompt(msgs)

    @staticmethod
    def _fallback_prompt(msgs):
        order = {"system": "system", "user": "user", "assistant": "assistant"}
        return (
            "".join(
                f"<|{order.get(m['role'], 'user')}|>\n{m['content']}\n" for m in msgs
            )
            + "<|assistant|>\n"
        )

    @staticmethod
    def _strip_special_tokens(txt: str) -> str:
        for t in ("<|eot_id|>", "</s>", "<|im_end|>"):
            txt = txt.replace(t, "")
        return txt.strip()


class NvidiaModel(LLMBase):
    """
    provider: nvidia
    Uses NVIDIA's OpenAI-compatible chat endpoint.
    """

    def __init__(
        self,
        model_name: str,
        temperature: float = 0.7,
        max_new_tokens: int = 1024,
        api_key: str = None,
        base_url: str = "https://integrate.api.nvidia.com/v1",
        timeout: int = 60,
    ):
        self.api_key = api_key or os.getenv("NVIDIA_API_KEY")
        if not self.api_key:
            raise ValueError(
                "NVIDIA API key must be provided either as an argument or via environment variable."
            )
        self.client = OpenAI(api_key=self.api_key, base_url=base_url, timeout=timeout)
        self.model_name = model_name
        self.temperature = temperature
        self.max_new_tokens = max_new_tokens

    async def chat(  # sync is fine, just wrap it in anyio.to_thread
        self,
        messages: List[Dict[str, str]],
        max_new_tokens: Optional[int] = None,
        temperature: Optional[float] = None,
    ) -> str:
        # unwrap kwargs you want to expose (temperature, max_tokens, stream…)
        resp = self.client.chat.completions.create(
            model=self.model_name,
            messages=messages,
            stream=False,
            temperature=temperature or self.temperature,
            max_tokens=max_new_tokens or self.max_new_tokens,
            top_p=1,
        )

        return resp.choices[0].message.content


_PROVIDER_MAP = {
    "openai": OpenAIModel,
    "local_hf": LocalHFModel,
    "mock": MockModel,
    "nvidia": NvidiaModel,
}


@lru_cache
def get_llm(profile: str) -> LLMBase:
    cfg = load_profiles()["profiles"][profile]
    cls = _PROVIDER_MAP[cfg["provider"]]
    return cls(**{k: v for k, v in cfg.items() if k != "provider"})
