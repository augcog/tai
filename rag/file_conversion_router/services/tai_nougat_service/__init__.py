import logging
import re
from pathlib import Path
from typing import List

from nougat import NougatModel
from nougat.postprocessing import markdown_compatible
from nougat.utils.checkpoint import get_checkpoint
from nougat.utils.dataset import LazyDataset
from nougat.utils.device import move_to_device
from torch.utils.data import ConcatDataset, DataLoader
from tqdm import tqdm

from .nougat_config import NougatConfig

logging.basicConfig(level=logging.INFO)


def create_model(config: NougatConfig) -> NougatModel:
    if config.checkpoint is None or not config.checkpoint.exists():
        config.checkpoint = get_checkpoint(config.checkpoint, model_tag=config.model_tag)

    model = NougatModel.from_pretrained(config.checkpoint)
    model = move_to_device(model, bf16=not config.full_precision, cuda=config.batch_size > 0)
    model.eval()
    return model


def load_datasets(config: NougatConfig, model: NougatModel) -> List[LazyDataset]:
    datasets = []
    for pdf in config.pdf_paths:
        if not pdf.exists():
            continue
        if config.output_dir:
            out_path = config.output_dir / pdf.with_suffix(".mmd").name
            if out_path.exists() and not config.recompute:
                logging.info(f"Skipping {pdf.name}, already computed. Run with recompute=True to convert again.")
                continue
        try:
            dataset = LazyDataset(
                pdf,
                model.encoder.prepare_input,
                config.pages,
            )
            datasets.append(dataset)
        except Exception as e:
            logging.info(f"Could not load file {str(pdf)}: {e}")
    return datasets


def process_output(output: str, page_num: int, config: NougatConfig) -> str:
    if output.strip() == "[MISSING_PAGE_POST]":
        return f"\n\n[MISSING_PAGE_EMPTY:{page_num}]\n\n"
    if config.markdown_compatible:
        output = markdown_compatible(output)
    return output


def main(config: NougatConfig):
    model = create_model(config)
    datasets = load_datasets(config, model)

    if not datasets:
        logging.info("No valid datasets found.")
        return

    dataloader = DataLoader(
        ConcatDataset(datasets),
        batch_size=config.batch_size,
        shuffle=False,
        collate_fn=LazyDataset.ignore_none_collate,
    )

    predictions = []
    file_index = 0
    page_num = 0

    for sample, is_last_page in tqdm(dataloader):
        model_output = model.inference(image_tensors=sample, early_stopping=config.skipping)

        for j, output in enumerate(model_output["predictions"]):
            if page_num == 0:
                logging.info(f"Processing file {datasets[file_index].name} with {datasets[file_index].size} pages")
            page_num += 1

            processed_output = process_output(output, page_num, config)
            predictions.append(processed_output)

            if is_last_page[j]:
                out = "".join(predictions).strip()
                out = re.sub(r"\n{3,}", "\n\n", out).strip()

                if config.output_dir:
                    out_path = config.output_dir / Path(is_last_page[j]).with_suffix(".mmd").name
                    out_path.parent.mkdir(parents=True, exist_ok=True)
                    out_path.write_text(out, encoding="utf-8")
                else:
                    print(out, "\n\n")

                predictions = []
                page_num = 0
                file_index += 1
