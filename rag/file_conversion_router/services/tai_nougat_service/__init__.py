from .config_nougat.tai_nougat_config import TAINougatConfig
from .mlx_nougat_service import api as mlx_nougat_api
from .torch_nougat_service import api as torch_nougat_api


def create_model(config: NougatConfig) -> NougatModel:
    if config.checkpoint is None or not config.checkpoint.exists():
        config.checkpoint = get_checkpoint(config.checkpoint, model_tag=config.model_tag)

    model = NougatModel.from_pretrained(config.checkpoint)
    model = move_to_device(model, bf16=not config.full_precision, cuda=config.batch_size > 0)
    model.eval()
    print("model loaded")  # debug
    return model


class NougatContainer(containers.DeclarativeContainer):
    model = providers.Singleton(
        create_model,
        config=NougatConfig()
    )


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
        output= f"Page {page_num}:\n{output.strip()}\n"
    return output


@inject
def main(config: NougatConfig, model: NougatModel = Provide[NougatContainer.model]):
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



def run_nougat(config: NougatConfig):
    """Run Nougat with the provided configuration.
    model is initialized using config only on the first time run_nougat is called .
    """
    input_path = config.pdf_paths[0]
    output_dir = config.output_dir

    using_torch = config.using_torch

    if using_torch:
        torch_nougat_api.convert_pdf_to_mmd(input_path, output_dir)
    else:
        mlx_nougat_api.convert_pdf_to_mmd(input_path, output_dir)
