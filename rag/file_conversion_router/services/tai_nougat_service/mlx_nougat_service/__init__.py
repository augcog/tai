"""Modified init code from the 'faster_nougat' package, to support more flexible configuration options.
"""

import os
from pathlib import Path
from typing import Tuple, Any

from tqdm import tqdm

from .faster_nougat import generate
from .faster_nougat.utils import get_model_and_processor, extract_pdf_pages_as_images
from ..config_nougat.tai_nougat_config import TAINougatConfig


class ModelNameMapper:
    tag_map = {
        "0.1.0-base": "facebook/nougat-base",
        "1.0.0-small": "facebook/nougat-small",
    }

    @classmethod
    def get_model_name(cls, tag: str) -> str:
        return cls.tag_map[tag]


class ModelManager:
    def __init__(self):
        self.model = None
        self.processor = None

    def get_model_and_processor(self) -> Tuple[Any, Any]:
        if self.model is None or self.processor is None:
            self.model, self.processor = get_model_and_processor(ModelNameMapper.
                                                                 get_model_name(TAINougatConfig.model_tag))
        return self.model, self.processor


model_manager = ModelManager()


def process_page(page_idx: int, image: Any, model: Any, processor: Any) -> str:
    pixel_values = processor(image, return_tensors="pt").pixel_values
    outputs = generate(model, pixel_values, max_new_tokens=4096, disable_tqdm=True)
    sequence = processor.batch_decode([outputs], skip_special_tokens=True)[0]
    sequence = processor.post_process_generation(sequence, fix_markdown=False)
    return f"\n\n# Page {page_idx + 1}\n\n{sequence}"


def main(input_pdf_path: Path, output_dir: Path) -> None:
    pdf_path = str(input_pdf_path)
    output_file_path = output_dir / f"{input_pdf_path.stem}.mmd"

    if not pdf_path.endswith(".pdf"):
        pdf_path += ".pdf"

    images = extract_pdf_pages_as_images(pdf_path)
    pages = list(range(len(images)))

    model, processor = model_manager.get_model_and_processor()

    full_content = ""
    for page_idx in tqdm(pages):
        image = images[page_idx]
        full_content += process_page(page_idx, image, model, processor)

    with open(output_file_path, "w") as f:
        f.write(full_content.strip())

    print(f"MMD content has been written to {output_file_path}")
