import yaml
import os
from enum import Enum
import torch
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Union, Any


# config/organizer.py
from pydantic import BaseModel, Field, ValidationError
from pathlib import Path
import yaml, os


class SummarizerCfg(BaseModel):
    max_length: int = Field(300, ge=50, le=4096)
    min_chunk_size: int = Field(100)
    llm_profile: str = "local"


class TopicClassifierCfg(BaseModel):
    max_topic_num: int = Field(3, ge=1)
    llm_profile: str = "accurate"


class FuncClassifierCfg(BaseModel):
    llm_profile: str = "local"


class OrganizerCfg(BaseModel):
    summarizer: SummarizerCfg = SummarizerCfg()
    topic_classifier: TopicClassifierCfg = TopicClassifierCfg()
    func_classifier: FuncClassifierCfg = FuncClassifierCfg()

    @classmethod
    def from_yaml(cls, path: str | os.PathLike) -> "OrganizerCfg":
        with open(path, "r", encoding="utf-8") as fh:
            raw = yaml.safe_load(fh)
        return cls(**raw)


def load_profiles() -> Dict[str, Any]:
    """
    Load model profiles from a YAML file.

    Returns:
        Dictionary of model profiles.
    """
    config_path = Path(__file__).parent.parent.parent / "models.yaml"
    if not config_path.exists():
        raise FileNotFoundError(f"Model profiles file not found: {config_path}")

    with open(config_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)
