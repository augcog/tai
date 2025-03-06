## File Conversion Router

The File Conversion Router seamlessly converts various supported file types within a specified source folder into desired formats, storing them in a designated destination folder. This scalable tool is designed for developers to extend; by implementing the abstract method `to_markdown` from the base class `BaseConverter`, the framework efficiently handles the conversion processes.

### Usage Instructions

#### Installation
Ensure Python version 3.10 is installed.
1. Install the required packages as specified in `requirements-base.txt` and `requirements-extend.txt` if you are using Linux or windows.
2. if you are using macOS, you need to install the mac_requirements.txt file.
3. if you have any problem relate to magic-pdf please refer to the following link: [magic-pdf](https://github.com/opendatalab/MinerU)
4. if you have GPU on your machine, you can also refer to https://github.com/opendatalab/MinerU to install the GPU version of the magic-pdf.
   ```bash
   - **Note**: Ensure the Transformer library version is below `4.39.0` as per compatibility requirements. The current version in `rag/requirements.txt` is set to `4.38.2`.
   - **Reference**: [Issue #1653 on GPT Academic](https://github.com/binary-husky/gpt_academic/issues/1653#issuecomment-2016794493)

#### Configuration
1. **Specify the Source Folder**: Define the folder containing the files to be converted.
2. **Set the Destination Folder**: Choose or create a folder where the converted files will be stored.

### Supported File Types
The router currently handles the following file types:
- `PDF`
- `md`
- `video`
  - Note: The logic for video conversion has been developed by Wayne, pending integration into the router through the `VideoConverter` class.
- `EdX`

### Output Formats
The following formats are generated in the destination folder for each supported input:
- Markdown (`md`)
- Text with Embedded Tree Structure (`tree.txt`)
- Serialized Data (`pkl`)

### Example Usage
The following example demonstrates the operational procedure of the File Conversion Router:

```python
from rag.file_conversion_router.api import convert_directory

input_path = "path/to/input/folder/file.pdf"
output_path = "path/to/output/folder"
convert_directory(input_path, output_path)
```

Execution of the above code will produce the following files in the output folder:
- `path/to/output/folder/file/file.md`
- `path/to/output/folder/file/file.md.tree.txt`
- `path/to/output/folder/file/file.md.pkl`

For more detailed examples and expected outputs, refer to the test suite located at `tests/test_file_conversion_router/test_api.py`.

### Advanced Features
- **Caching Mechanism**: This feature enhances efficiency by avoiding the re-conversion of files with unchanged content.
- **Performance Monitoring**: Logs and monitors the time taken for each file conversion task.
- **Conversion Fidelity**: Depending on the processing device's capabilities (CPU, GPU with MPS or CUDA technologies), the conversion results might slightly vary. To accommodate this, a similarity threshold (`SIMILARITY_THRESHOLD `) of 95% is set for test validations. Files meeting or exceeding this threshold are considered successfully converted; those that do not are flagged for review, with discrepancies detailed in the output logs.
- **Robust Logging**: Provides detailed records of each conversion process, aiding in troubleshooting and performance assessment.
