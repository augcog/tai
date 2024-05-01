## File Conversion Router

The File Conversion Router efficiently converts various supported file types within a specified source folder into desired formats, storing them in a designated destination folder. This tool is designed for scalability; developers simply need to implement the abstract method `to_markdown` from the base class `BaseConverter`, and the framework handles the rest.

### Usage Instructions

#### Installation
Ensure Python version 3.9 or higher is installed.
1. Install the required packages listed in `requirements.txt`.
> Special Note: Transformer version needs to be lower than `4.39.0`. Current `rag/requirements.txt` has the version set to `4.38.2`.
> 
> Reference: https://github.com/binary-husky/gpt_academic/issues/1653#issuecomment-2016794493

#### Configuration
1. **Specify the Source Folder**: Identify the folder containing the files to be converted.
2. **Set the Destination Folder**: Select or create a folder to store the converted files.

### Supported File Types
The router currently supports the following file types:
- `pdf`
- `md`
- `video`
  - Incoming. Wayne had implemented the logic, it's pending for him to integrate it into the router by implementing the `to_markdown` method in the `VideoConverter` class.

### Output Formats
The router generates the following files in the destination folder for each supported input type:
- Markdown (`md`)
- Text with Tree Structure Embedded (`tree.txt`)
- Pickle (`pkl`)

### Example Usage
Below is an example demonstrating how to use the File Conversion Router:

```python
from rag.file_conversion_router.api import convert_directory

input_path = "path/to/input/folder/file.pdf"
output_path = "path/to/output/folder"
convert_directory(input_path, output_path)
```

After running the above code, the following files will be generated in the output folder:
- `path/to/output/folder/file/file.md`
- `path/to/output/folder/file/file_tree.txt`
- `path/to/output/folder/file/file.pkl`

For detailed usage examples and expected outputs, refer to the test cases: `tests/test_file_conversion_router/test_api.py`

### Additional Features
- **Time Measurement**: Monitors and logs the time taken for each conversion task.
- **Logging**: Records the status of each file conversion process.
- **Caching**: Prevents re-conversion by caching already converted files.
