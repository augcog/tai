"""
PDF to Image Converter

This utility converts PDF files to images (PNG/JPG) with support for:
- Single PDF to multiple images (one per page)
- Batch conversion of multiple PDFs
- Configurable DPI, format, and output directory
- Progress tracking and error handling
"""

import fitz  # PyMuPDF
from pathlib import Path
from typing import Union, List, Optional, Literal
from PIL import Image
import io
from tqdm import tqdm


ImageFormat = Literal["png", "jpg", "jpeg"]


def pdf_to_images(
    pdf_path: Union[str, Path],
    output_dir: Union[str, Path],
    dpi: int = 300,
    image_format: ImageFormat = "png",
    prefix: Optional[str] = None,
    page_range: Optional[tuple[int, int]] = None,
) -> List[Path]:
    """
    Convert a single PDF file to images.

    Args:
        pdf_path: Path to the PDF file
        output_dir: Directory to save the images
        dpi: Resolution in dots per inch (default: 300)
        image_format: Output image format - "png", "jpg", or "jpeg" (default: "png")
        prefix: Custom prefix for output filenames (default: PDF filename)
        page_range: Tuple of (start_page, end_page) to convert specific pages (1-indexed)
                   If None, converts all pages

    Returns:
        List of paths to generated image files

    Example:
        >>> images = pdf_to_images("lecture.pdf", "output/", dpi=150)
        >>> print(images)
        [PosixPath('output/lecture_page_001.png'), PosixPath('output/lecture_page_002.png')]
    """
    pdf_path = Path(pdf_path)
    output_dir = Path(output_dir)

    if not pdf_path.exists():
        raise FileNotFoundError(f"PDF file not found: {pdf_path}")

    output_dir.mkdir(parents=True, exist_ok=True)

    # Normalize image format
    if image_format.lower() in ["jpg", "jpeg"]:
        image_format = "jpeg"
        ext = "jpg"
    else:
        image_format = "png"
        ext = "png"

    # Use custom prefix or PDF filename
    file_prefix = prefix if prefix else pdf_path.stem

    # Open PDF
    doc = fitz.open(pdf_path)
    total_pages = len(doc)

    # Determine page range
    if page_range:
        start_page = max(1, page_range[0]) - 1  # Convert to 0-indexed
        end_page = min(total_pages, page_range[1])
    else:
        start_page = 0
        end_page = total_pages

    # Calculate zoom factor for desired DPI
    # PyMuPDF default is 72 DPI
    zoom = dpi / 72.0
    mat = fitz.Matrix(zoom, zoom)

    generated_images = []

    print(f"Converting {pdf_path.name}: {end_page - start_page} pages at {dpi} DPI...")

    for page_num in tqdm(range(start_page, end_page), desc="Pages"):
        page = doc[page_num]

        # Render page to pixmap
        pix = page.get_pixmap(matrix=mat)

        # Convert to PIL Image for format flexibility
        img_data = pix.tobytes(image_format)
        img = Image.open(io.BytesIO(img_data))

        # Generate output filename with zero-padded page number
        page_display_num = page_num + 1
        output_filename = f"{file_prefix}_page_{page_display_num:03d}.{ext}"
        output_path = output_dir / output_filename

        # Save image
        if image_format == "jpeg":
            img.save(output_path, "JPEG", quality=95)
        else:
            img.save(output_path, "PNG")

        generated_images.append(output_path)

    doc.close()

    print(f"✓ Generated {len(generated_images)} images in {output_dir}")
    return generated_images


def batch_pdf_to_images(
    pdf_paths: List[Union[str, Path]],
    output_dir: Union[str, Path],
    dpi: int = 300,
    image_format: ImageFormat = "png",
    create_subdirs: bool = True,
) -> dict[Path, List[Path]]:
    """
    Convert multiple PDF files to images in batch.

    Args:
        pdf_paths: List of PDF file paths
        output_dir: Base directory to save images
        dpi: Resolution in dots per inch (default: 300)
        image_format: Output image format - "png", "jpg", or "jpeg" (default: "png")
        create_subdirs: If True, creates a subdirectory for each PDF's images

    Returns:
        Dictionary mapping PDF path to list of generated image paths

    Example:
        >>> pdfs = ["lecture1.pdf", "lecture2.pdf"]
        >>> results = batch_pdf_to_images(pdfs, "output/")
        >>> print(results)
        {PosixPath('lecture1.pdf'): [...], PosixPath('lecture2.pdf'): [...]}
    """
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    results = {}

    for pdf_path in pdf_paths:
        pdf_path = Path(pdf_path)

        if not pdf_path.exists():
            print(f"⚠ Skipping {pdf_path}: File not found")
            continue

        try:
            # Create subdirectory for this PDF if requested
            if create_subdirs:
                pdf_output_dir = output_dir / pdf_path.stem
            else:
                pdf_output_dir = output_dir

            images = pdf_to_images(
                pdf_path=pdf_path,
                output_dir=pdf_output_dir,
                dpi=dpi,
                image_format=image_format,
            )

            results[pdf_path] = images

        except Exception as e:
            print(f"✗ Error converting {pdf_path}: {e}")
            results[pdf_path] = []

    return results


def pdf_directory_to_images(
    input_dir: Union[str, Path],
    output_dir: Union[str, Path],
    dpi: int = 300,
    image_format: ImageFormat = "png",
    recursive: bool = False,
) -> dict[Path, List[Path]]:
    """
    Convert all PDFs in a directory to images.

    Args:
        input_dir: Directory containing PDF files
        output_dir: Directory to save images
        dpi: Resolution in dots per inch (default: 300)
        image_format: Output image format - "png", "jpg", or "jpeg" (default: "png")
        recursive: If True, searches subdirectories for PDFs

    Returns:
        Dictionary mapping PDF path to list of generated image paths

    Example:
        >>> results = pdf_directory_to_images("pdfs/", "images/", dpi=200)
    """
    input_dir = Path(input_dir)

    if not input_dir.exists() or not input_dir.is_dir():
        raise NotADirectoryError(f"Input directory not found: {input_dir}")

    # Find all PDFs
    if recursive:
        pdf_files = list(input_dir.rglob("*.pdf"))
    else:
        pdf_files = list(input_dir.glob("*.pdf"))

    if not pdf_files:
        print(f"No PDF files found in {input_dir}")
        return {}

    print(f"Found {len(pdf_files)} PDF file(s)")

    return batch_pdf_to_images(
        pdf_paths=pdf_files,
        output_dir=output_dir,
        dpi=dpi,
        image_format=image_format,
        create_subdirs=True,
    )


# CLI interface
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="Convert PDF files to images",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Convert single PDF
  python pdf_to_image.py input.pdf --output images/

  # Convert with custom DPI and format
  python pdf_to_image.py input.pdf --output images/ --dpi 150 --format jpg

  # Convert directory of PDFs
  python pdf_to_image.py input_dir/ --output images/ --directory

  # Convert specific page range
  python pdf_to_image.py input.pdf --output images/ --pages 1 10
        """
    )

    parser.add_argument("input", help="PDF file or directory path")
    parser.add_argument("--output", "-o", required=True, help="Output directory")
    parser.add_argument("--dpi", type=int, default=300, help="DPI resolution (default: 300)")
    parser.add_argument("--format", "-f", choices=["png", "jpg", "jpeg"],
                       default="png", help="Image format (default: png)")
    parser.add_argument("--directory", "-d", action="store_true",
                       help="Treat input as directory of PDFs")
    parser.add_argument("--recursive", "-r", action="store_true",
                       help="Search subdirectories (with --directory)")
    parser.add_argument("--prefix", help="Custom filename prefix")
    parser.add_argument("--pages", nargs=2, type=int, metavar=("START", "END"),
                       help="Page range to convert (1-indexed)")

    args = parser.parse_args()

    try:
        if args.directory:
            results = pdf_directory_to_images(
                input_dir=args.input,
                output_dir=args.output,
                dpi=args.dpi,
                image_format=args.format,
                recursive=args.recursive,
            )
            print(f"\n✓ Processed {len(results)} PDF file(s)")
        else:
            page_range = tuple(args.pages) if args.pages else None
            images = pdf_to_images(
                pdf_path=args.input,
                output_dir=args.output,
                dpi=args.dpi,
                image_format=args.format,
                prefix=args.prefix,
                page_range=page_range,
            )
            print(f"\n✓ Generated {len(images)} image(s)")

    except Exception as e:
        print(f"\n✗ Error: {e}")
        exit(1)
