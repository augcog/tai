# Operations Log Format

The `operations_log.json` file is automatically generated when using the `--create-folders` option with `folder_structure_generator.py`. It provides a detailed record of all operations performed or planned.

## Structure

```json
{
  "operation": "folder_structure_creation",
  "mode": "dry_run" | "execute",
  "base_directory": "/path/to/output",
  "course_id": "CS61A",
  "term": "2025FA",
  "total_units": 11,
  "folders": [
    {
      "unit_id": "U01",
      "folder_name": "U01_Introduction_&_Python_Basics",
      "folder_path": "/path/to/output/U01_Introduction_&_Python_Basics",
      "title": "Introduction & Python Basics",
      "description": "Covers course introduction, Python syntax, variables...",
      "aliases": ["intro", "welcome", "01", "control", "02", "03"],
      "expected_types": ["lecture_slide", "reading", "notes", "code"],
      "suggested_files": [
        "assets/slides/01-Welcome_1pp/01-Welcome_1pp.pdf.md",
        "assets/slides/03-Control_1pp/03-Control_1pp.pdf.md"
      ],
      "file_count": 6,
      "actions": [
        {
          "type": "create_directory",
          "path": "/path/to/output/U01_Introduction_&_Python_Basics",
          "status": "planned" | "success" | "failed"
        },
        {
          "type": "create_readme",
          "path": "/path/to/output/U01_Introduction_&_Python_Basics/README.md",
          "status": "planned" | "success" | "failed"
        }
      ]
    }
  ]
}
```

## Field Descriptions

### Top Level
- **operation**: Type of operation (always "folder_structure_creation")
- **mode**: "dry_run" (preview mode) or "execute" (actual creation)
- **base_directory**: Root directory where folders are created
- **course_id**: Course identifier
- **term**: Academic term
- **total_units**: Number of units/folders created
- **folders**: Array of folder information

### Folder Object
- **unit_id**: Unique unit identifier (U01, U02, etc.)
- **folder_name**: Physical folder name on disk
- **folder_path**: Full absolute path to folder
- **title**: Human-readable unit title
- **description**: Brief description of unit content
- **aliases**: Keywords used to match files to this unit
- **expected_types**: Types of materials expected in this unit
- **suggested_files**: List of files that should belong to this unit
- **file_count**: Number of suggested files
- **actions**: Array of operations performed

### Action Object
- **type**: Type of action performed
  - `create_directory`: Folder creation
  - `create_readme`: README.md creation
  - `error`: Error encountered
- **path**: Path to created resource
- **status**: Operation status
  - `planned`: Dry run mode, not executed
  - `success`: Successfully executed
  - `failed`: Execution failed
- **message**: (Optional) Error message if status is "failed"

## Use Cases

### 1. Reviewing Planned Structure (Dry Run)
When using `--dry-run`, the operations log shows what **would** be created:
- All actions have status "planned"
- No actual file system changes are made
- Review the structure before executing

### 2. Verifying Execution
After actual folder creation (without `--dry-run`):
- All actions show "success" or "failed" status
- Provides audit trail of what was created
- Helps diagnose any creation errors

### 3. File Organization Planning
Use the `suggested_files` arrays to understand:
- Which files belong to which unit
- How the LLM categorized your course materials
- Potential file organization strategy

### 4. Automation and Scripting
The operations log JSON can be:
- Parsed by scripts to move files to appropriate folders
- Used to generate reports or documentation
- Integrated into CI/CD pipelines

## Example Usage

```bash
# Generate structure and log without creating folders
python folder_structure_generator.py \
  --scan-dir /path/to/course \
  --course-id CS61A \
  --term 2025FA \
  --output syllabus.json \
  --create-folders \
  --output-dir /path/to/organized \
  --dry-run

# Review operations_log.json to verify structure
cat /path/to/organized/operations_log.json

# Execute if satisfied with structure
python folder_structure_generator.py \
  --scan-dir /path/to/course \
  --course-id CS61A \
  --term 2025FA \
  --output syllabus.json \
  --create-folders \
  --output-dir /path/to/organized

# Review final operations_log.json to confirm success
cat /path/to/organized/operations_log.json
```

## Integration with File Movement

The operations log can be used to automate file movement:

```python
import json
import shutil
from pathlib import Path

# Load operations log
with open("operations_log.json") as f:
    log = json.load(f)

base_dir = Path(log["base_directory"])

# Move files to appropriate folders
for folder in log["folders"]:
    dest_folder = Path(folder["folder_path"])

    for file_path in folder["suggested_files"]:
        source = base_dir.parent / file_path  # Adjust as needed
        dest = dest_folder / Path(file_path).name

        if source.exists():
            shutil.copy2(source, dest)
            print(f"Moved: {file_path} -> {dest}")
```

## Notes

- The operations log is always generated when `--create-folders` is used
- Default location: `operations_log.json` in the output directory
- Custom location: Use `--operations-log /path/to/custom_log.json`
- Both dry run and execution modes generate the log with different statuses
