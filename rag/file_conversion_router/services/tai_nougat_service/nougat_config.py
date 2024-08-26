from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

from nougat.utils.device import default_batch_size


@dataclass
class NougatConfig:
    batch_size: int = default_batch_size()
    checkpoint: Optional[Path] = None
    model_tag: str = "0.1.0-small"
    output_dir: Optional[Path] = None
    recompute: bool = True
    full_precision: bool = False
    markdown_compatible: bool = True
    skipping: bool = False
    pages: Optional[str] = None
    pdf_paths: List[Path] = None
