#!/usr/bin/env python3
"""
Folder Structure Generator - Uses LLM to analyze course files and generate organized folder structure.

This module analyzes existing course files and uses an LLM to:
1. Determine how many folders should be created
2. Decide the folder structure (topics/units/modules)
3. Generate a syllabus.json file with the recommended structure
4. Optionally create the physical folder structure

USAGE AS LIBRARY:
    from folder_structure_generator import generate_folder_structure

    syllabus = generate_folder_structure(
        scan_dir="/path/to/course/materials",
        course_id="CS61A",
        term="2025FA",
        output="syllabus.json",
        create_folders=True,
        output_dir="/path/to/organized/folders",
        dry_run=True
    )

USAGE AS SCRIPT:
    1. Edit the configuration dictionary in main() function
    2. Run: python folder_structure_generator.py

API REFERENCE:
    generate_folder_structure() - Main API function for generating folder structures
    create_folder_structure() - Create physical folders from syllabus JSON
    move_files_to_folders() - Move files into organized folder structure
    record_file_statistics() - Record statistics about ungrouped and not-found files
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path
from typing import Dict, Any, List, Set
import fnmatch
from dotenv import load_dotenv
# try:
#     # Try multiple possible locations for .env file
#     possible_env_paths = [
#         Path(__file__).parent.parent / '.env',
#         Path(__file__).parent.parent.parent / '.env',
#         Path.cwd() / '.env',
#         Path.cwd().parent / '.env',
#     ]
#
#     for env_path in possible_env_paths:
#         if env_path.exists():
#             load_dotenv(env_path)
#             break
#
# except ImportError:
#     pass

load_dotenv()
api_key = os.getenv("OPENAI_API_KEY")


FOLDER_STRUCTURE_PROMPT = """
You are an expert educational content organizer. Analyze the course files below and generate a CONTENT-BASED folder organization system.

### Task
Organize course materials by CONTENT CATEGORY, where each category can contain MULTIPLE FILE TYPES.

**KEY PRINCIPLE**: Group related materials together regardless of file type.
- Example: "Lectures" unit contains BOTH lecture slides (.pdf) AND lecture videos (.mp4)
- Example: "Homeworks" unit contains BOTH homework PDFs AND solution notebooks

### Course Information
Course ID: {course_id}
Term: {term}
Total Files: {file_count}

### File List
{file_list}

### Output Requirements
Generate a JSON object with this exact structure (DO NOT include suggested_files - we will map files separately):

{{
  "course_id": "{course_id}",
  "term": "{term}",
  "structure_type": "content_based",
  "units": [
    {{
      "unit_id": "unique_identifier",
      "title": "Category Title",
      "aliases": ["keyword1", "keyword2", "pattern1", "pattern2"],
      "description": "Description of what belongs in this category",
      "expected_types": ["file_type1", "file_type2"],
      "merge_related": true
    }}
  ]
}}

### Instructions - CONTENT-BASED ORGANIZATION

Analyze the file list and identify natural content groupings:

1. **Group by CONTENT, not FILE TYPE**:
   - ✅ GOOD: "Lectures" (contains slides.pdf + videos.mp4 + notes.md)
   - ❌ BAD: "Lecture_Slides" and "Lecture_Videos" separately

   - ✅ GOOD: "Homeworks" (contains problems.pdf + solutions.ipynb + starter_code.py)
   - ❌ BAD: "Homework_PDFs" and "Homework_Notebooks" separately

2. **Common Content Categories** (adapt based on what you find):
   - **Lectures**: All lecture-related materials (slides, videos, recordings, transcripts)
   - **Discussions**: Discussion materials (slides, videos, worksheets)
   - **Homeworks**: All homework materials (PDFs, notebooks, solutions, starter code)
   - **Labs**: Lab materials (instructions, notebooks, data, solutions)
   - **Projects**: Project materials (specs, code, documentation)
   - **Exams**: Exam materials (practice exams, solutions, study guides)
   - **Resources**: Course resources (syllabus, schedules, reference materials)
   - **Documentation**: Technical documentation (API docs, guides, tutorials)

3. **Identify Relationships Across File Types**:
   - Look for files with matching dates (e.g., "10-10 Lecture.pdf" + "10-10 Lecture.mp4")
   - Look for files with matching numbers (e.g., "hw01.pdf" + "hw01_solution.ipynb")
   - Look for files with matching topics (e.g., "ARUCO_slides.pdf" + "ARUCO_video.mp4")
   - Look for directory names that indicate content (e.g., "Lecture_Slides_2024" + "Lecture_Videos_2024" → both go to "Lectures")

4. **Generate Comprehensive Aliases**:
   - Include keywords from ALL related file types
   - Example: "Lectures" aliases = ["lecture", "slides", "video", "recording", "Lecture_Slides", "Lecture_Videos"]
   - Include directory names that contain this content
   - Include numbering patterns (01, 02, 03)
   - Include date patterns if present
   - DO NOT hardcode generic examples - extract from actual files

5. **Set expected_types to ALL file types in category**:
   - Example: "Lectures" → expected_types: ["pdf", "mp4", "mov", "pptx"]
   - This helps Stage 3 & 4 understand that multiple types belong together

### Important Notes
- **CRITICAL**: Merge related file types into single categories (lectures with videos, homeworks with notebooks)
- Create categories based on what actually exists in the files
- Focus on generating ALIASES that capture ALL variations of the content
- Students will find materials by content (Lecture 1, Homework 2) not by type (Slides, Videos)

Output **only valid JSON** with no additional text or explanations.
"""


def load_ignore_patterns(ignore_file: str | Path) -> Set[str]:
    """
    Load ignore patterns from a file (similar to .gitignore format).

    Supports:
    - Line comments starting with #
    - Glob patterns (*.txt, test_*)
    - Directory patterns (dir/, **/cache/)
    - Negation patterns (!important.txt)

    Args:
        ignore_file: Path to the ignore file

    Returns:
        Set of ignore patterns

    Example ignore file:
        # Ignore all hidden files
        .*

        # Ignore specific directories
        __pycache__/
        .git/
        node_modules/

        # Ignore file patterns
        *.pyc
        *.log
        *~

        # Ignore specific files
        .DS_Store
        Thumbs.db
    """
    patterns = set()
    ignore_path = Path(ignore_file)

    if not ignore_path.exists():
        return patterns

    try:
        with open(ignore_path, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                # Skip empty lines and comments
                if not line or line.startswith('#'):
                    continue
                patterns.add(line)
    except Exception as e:
        print(f"Warning: Failed to read ignore file {ignore_file}: {e}")

    return patterns


def should_ignore(path: Path, base_dir: Path, patterns: Set[str]) -> bool:
    """
    Check if a path should be ignored based on patterns.

    Args:
        path: Path to check
        base_dir: Base directory for relative path calculation
        patterns: Set of ignore patterns

    Returns:
        True if path should be ignored, False otherwise
    """
    if not patterns:
        return False

    # Get relative path
    try:
        rel_path = path.relative_to(base_dir)
    except ValueError:
        return False

    rel_path_str = str(rel_path)
    name = path.name

    for pattern in patterns:
        # Handle negation patterns (not implemented yet, but prepared)
        negate = pattern.startswith('!')
        if negate:
            pattern = pattern[1:]

        # Check if pattern matches directory
        if pattern.endswith('/'):
            # Directory pattern
            dir_pattern = pattern.rstrip('/')
            if path.is_dir():
                if fnmatch.fnmatch(name, dir_pattern):
                    return True
                # Check if any parent directory matches
                for parent in path.parents:
                    if parent == base_dir:
                        break
                    if fnmatch.fnmatch(parent.name, dir_pattern):
                        return True
        else:
            # File or path pattern
            # Check full relative path
            if fnmatch.fnmatch(rel_path_str, pattern):
                return True
            # Check just the filename
            if fnmatch.fnmatch(name, pattern):
                return True
            # Check with ** glob pattern
            if '**' in pattern:
                pattern_parts = pattern.split('**')
                if len(pattern_parts) == 2:
                    prefix, suffix = pattern_parts
                    if rel_path_str.startswith(prefix.rstrip('/')) and rel_path_str.endswith(suffix.lstrip('/')):
                        return True

    return False


def scan_directory(directory: str | Path, ignore_file: str | Path | None = None) -> tuple[List[Dict[str, str]], int]:
    """
    Scan a directory and return list of files with metadata.

    Args:
        directory: Directory to scan
        ignore_file: Optional path to ignore patterns file (similar to .gitignore)
                    If None, will look for '.scanignore' in the scan directory

    Returns:
        Tuple of (list of file dicts, count of ignored files)
        File dicts have keys: name, path, extension, size_kb
    """
    directory = Path(directory)
    if not directory.exists() or not directory.is_dir():
        raise ValueError(f"Invalid directory: {directory}")

    # Load ignore patterns
    patterns = set()
    if ignore_file is None:
        # Check for default .scanignore file in the directory
        default_ignore = directory / '.scanignore'
        if default_ignore.exists():
            patterns = load_ignore_patterns(default_ignore)
            print(f"Loaded {len(patterns)} ignore patterns from {default_ignore}")
    elif ignore_file:
        patterns = load_ignore_patterns(ignore_file)
        print(f"Loaded {len(patterns)} ignore patterns from {ignore_file}")

    files = []
    ignored_count = 0

    for file_path in directory.rglob("*"):
        if file_path.is_file():
            # Skip hidden files and system files (unless explicitly not ignored)
            if file_path.name.startswith('.'):
                ignored_count += 1
                continue

            # Check ignore patterns
            if should_ignore(file_path, directory, patterns):
                ignored_count += 1
                continue

            files.append({
                "name": file_path.name,
                "path": str(file_path.relative_to(directory)),
                "extension": file_path.suffix,
                "size_kb": file_path.stat().st_size // 1024
            })

    if ignored_count > 0:
        print(f"Ignored {ignored_count} files based on patterns")

    return sorted(files, key=lambda x: x["name"]), ignored_count


def format_file_list(files: List[Dict[str, str]], max_files: int = 300) -> str:
    """
    Format file list for LLM prompt.

    For large file lists, provides a summary and representative sample.
    """
    if len(files) <= max_files:
        lines = []
        for f in files:
            lines.append(f"- {f['name']} ({f['extension']}, {f['size_kb']}KB) at {f['path']}")
        return "\n".join(lines)

    # For large file lists, provide statistics and samples
    from collections import Counter
    import re

    ext_counts = Counter(f['extension'] for f in files)

    # Analyze directory structure patterns
    dir_patterns = Counter()
    for f in files:
        path_parts = Path(f['path']).parts
        if len(path_parts) > 1:
            dir_patterns[path_parts[0]] += 1

    summary = [
        f"TOTAL FILES: {len(files)} (showing representative sample)",
        f"\n⚠️ IMPORTANT: You are seeing only {max_files} files out of {len(files)} total.",
        f"You MUST extrapolate patterns to capture ALL {len(files)} files!",
        f"\nFILE TYPE DISTRIBUTION:",
    ]

    for ext, count in ext_counts.most_common():
        summary.append(f"  {ext or '(no extension)'}: {count} files")

    summary.append(f"\nTOP-LEVEL DIRECTORY DISTRIBUTION:")
    for dir_name, count in dir_patterns.most_common(20):
        summary.append(f"  {dir_name}/: {count} files")

    # Identify numbered patterns
    summary.append(f"\nPATTERN ANALYSIS:")
    numbered_patterns = {}
    for f in files:
        path = f['path']
        # Look for number patterns in paths
        numbers = re.findall(r'\d+', path)
        if numbers:
            # Create pattern by replacing numbers with placeholder (use <N> to avoid format conflicts)
            pattern = re.sub(r'\d+', '<N>', path)
            if pattern not in numbered_patterns:
                numbered_patterns[pattern] = {'count': 0, 'numbers': set()}
            numbered_patterns[pattern]['count'] += 1
            numbered_patterns[pattern]['numbers'].update(numbers)

    # Show top patterns
    for pattern, data in sorted(numbered_patterns.items(), key=lambda x: x[1]['count'], reverse=True)[:10]:
        num_range = f"[{min(data['numbers'])}-{max(data['numbers'])}]" if data['numbers'] else ""
        summary.append(f"  Pattern: {pattern} (appears {data['count']}x, numbers: {num_range})")

    summary.append(f"\n\nREPRESENTATIVE SAMPLE (first {max_files} files by name):")

    for f in files[:max_files]:
        summary.append(f"- {f['name']} ({f['extension']}, {f['size_kb']}KB) at {f['path']}")

    summary.append(f"\n... and {len(files) - max_files} more files")
    summary.append(f"\n⚠️ REMINDER: Use the pattern analysis above to capture ALL {len(files)} files!")

    return "\n".join(summary)


def call_openai(prompt: str, model: str = "gpt-4.1", use_structured_output: bool = False, json_schema: Dict[str, Any] | None = None) -> str:
    """
    Call OpenAI API with JSON response format.

    Args:
        prompt: The prompt to send
        model: Model name
        use_structured_output: If True, use structured output with JSON schema
        json_schema: JSON schema for structured output (required if use_structured_output=True)
    """
    from openai import OpenAI

    api_key = os.getenv("OPENAI_API_KEY")
    if not api_key:
        raise RuntimeError("OPENAI_API_KEY not set in environment")

    client = OpenAI(api_key=api_key)

    try:
        if use_structured_output and json_schema:
            # Use structured output with JSON schema enforcement
            resp = client.chat.completions.create(
                model=model,
                messages=[{"role": "user", "content": prompt}],
                temperature=0.3,
                max_tokens=16000,
                response_format={
                    "type": "json_schema",
                    "json_schema": json_schema
                }
            )
        else:
            # Standard JSON object response
            resp = client.chat.completions.create(
                model=model,
                messages=[{"role": "user", "content": prompt}],
                temperature=0.3,
                max_tokens=16000,
                response_format={"type": "json_object"}
            )
        return resp.choices[0].message.content.strip()
    except Exception as e:
        raise RuntimeError(f"OpenAI API call failed: {e}")


def extract_json(text: str) -> Dict[str, Any]:
    """Extract JSON from LLM response."""
    import re

    text = text.strip()

    # Try parsing as-is (for structured output)
    try:
        return json.loads(text)
    except json.JSONDecodeError as e:
        # If it fails, try to get more details
        print(f"JSON parse error at position {e.pos}: {e.msg}")
        # Try to fix common issues
        pass

    # Extract from code block
    patterns = [
        r"```json\s*(\{.*?\})\s*```",
        r"```\s*(\{.*?\})\s*```",
    ]
    for pattern in patterns:
        m = re.search(pattern, text, flags=re.DOTALL)
        if m:
            try:
                return json.loads(m.group(1))
            except json.JSONDecodeError:
                continue

    # Find first complete JSON object by brace matching
    start = text.find('{')
    if start != -1:
        depth = 0
        in_string = False
        escape = False

        for i in range(start, len(text)):
            char = text[i]

            # Handle escape sequences
            if escape:
                escape = False
                continue
            if char == '\\':
                escape = True
                continue

            # Handle strings
            if char == '"':
                in_string = not in_string
                continue

            # Only count braces outside strings
            if not in_string:
                if char == '{':
                    depth += 1
                elif char == '}':
                    depth -= 1
                    if depth == 0:
                        try:
                            json_str = text[start:i+1]
                            return json.loads(json_str)
                        except json.JSONDecodeError as e:
                            print(f"Failed to parse extracted JSON: {e}")
                            # Show context around error
                            error_pos = e.pos
                            context_start = max(0, error_pos - 50)
                            context_end = min(len(json_str), error_pos + 50)
                            print(f"Context: ...{json_str[context_start:context_end]}...")
                        break

    # Last resort: save to file for debugging
    debug_file = Path("debug_response.txt")
    debug_file.write_text(text, encoding='utf-8')
    print(f"Full response saved to {debug_file} for debugging")

    raise ValueError(f"No valid JSON found in response. Preview: {text[:200]}...")


def record_file_statistics(
    syllabus: Dict[str, Any],
    move_log: Dict[str, Any] | None = None,
    output_path: str | Path = "file_statistics.json"
) -> Dict[str, Any]:
    """
    Record detailed statistics about all files that were not successfully moved.

    Args:
        syllabus: Syllabus JSON structure with ungrouped_files list
        move_log: Optional move log from move_files_to_folders() operation
        output_path: Path where statistics JSON will be saved

    Returns:
        Dictionary containing file statistics

    Example output:
        {
            "course_id": "CS61A",
            "term": "2025FA",
            "timestamp": "2025-10-09T10:30:00",
            "unmoved_files": {
                "count": 25,
                "by_reason": {
                    "ungrouped": 15,
                    "not_found": 8,
                    "failed": 2
                },
                "details": [
                    {
                        "file_path": "lectures/01.pdf",
                        "reason": "not_found",
                        "message": "Source file does not exist",
                        "unit_id": "lectures",
                        "unit_title": "Lectures"
                    },
                    ...
                ]
            }
        }
    """
    from datetime import datetime

    # Collect all unmoved files with reasons
    unmoved_files = []
    reason_counts = {"ungrouped": 0, "not_found": 0, "failed": 0}

    # 1. Ungrouped files - files not assigned to any unit
    ungrouped = syllabus.get("ungrouped_files", [])
    for file_path in ungrouped:
        unmoved_files.append({
            "file_path": file_path,
            "reason": "ungrouped",
            "message": "File was not categorized into any unit during analysis",
            "unit_id": None,
            "unit_title": None
        })
        reason_counts["ungrouped"] += 1

    # 2. Not found files - files assigned to units but missing from source
    if move_log:
        # Extract not found files
        not_found = move_log.get("lost_files", [])
        for item in not_found:
            if isinstance(item, dict):
                unmoved_files.append({
                    "file_path": item.get("file_path"),
                    "reason": "not_found",
                    "message": "Source file does not exist in the scan directory",
                    "unit_id": item.get("unit_id"),
                    "unit_title": item.get("unit_title")
                })
            else:
                unmoved_files.append({
                    "file_path": str(item),
                    "reason": "not_found",
                    "message": "Source file does not exist in the scan directory",
                    "unit_id": None,
                    "unit_title": None
                })
            reason_counts["not_found"] += 1

        # 3. Failed files - files that existed but failed to move
        for unit_log in move_log.get("units", []):
            for operation in unit_log.get("operations", []):
                if operation.get("status") == "failed":
                    unmoved_files.append({
                        "file_path": Path(operation.get("source", "")).name,
                        "reason": "failed",
                        "message": f"Move operation failed: {operation.get('message', 'Unknown error')}",
                        "unit_id": unit_log.get("unit_id"),
                        "unit_title": unit_log.get("folder_name"),
                        "error_details": operation.get("message")
                    })
                    reason_counts["failed"] += 1

    # Create statistics structure
    statistics = {
        "course_id": syllabus.get("course_id"),
        "term": syllabus.get("term"),
        "timestamp": datetime.now().isoformat(),
        "unmoved_files": {
            "count": len(unmoved_files),
            "description": "All files that were not successfully moved to organized folders",
            "by_reason": reason_counts,
            "details": unmoved_files
        },
        "summary": {
            "total_files_scanned": sum(len(unit.get('suggested_files', [])) for unit in syllabus.get('units', [])) + len(ungrouped),
            "total_unmoved": len(unmoved_files),
            "ungrouped": reason_counts["ungrouped"],
            "not_found": reason_counts["not_found"],
            "failed": reason_counts["failed"],
            "successfully_moved": move_log.get("total_files_moved", 0) if move_log else 0
        }
    }

    # Save to JSON file
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(statistics, f, ensure_ascii=False, indent=2)

    # Print detailed summary
    print(f"\n{'='*60}")
    print("FILE STATISTICS REPORT")
    print(f"{'='*60}")
    print(f"Course: {statistics['course_id']} ({statistics['term']})")
    print(f"\nTotal Files: {statistics['summary']['total_files_scanned']}")
    print(f"Successfully Moved: {statistics['summary']['successfully_moved']}")
    print(f"Unmoved Files: {statistics['unmoved_files']['count']}")

    print(f"\n{'Breakdown by Reason:'}")
    print(f"  - Ungrouped: {reason_counts['ungrouped']} (not categorized into any unit)")
    print(f"  - Not Found: {reason_counts['not_found']} (assigned but missing from source)")
    print(f"  - Failed: {reason_counts['failed']} (move operation failed)")

    # Show detailed examples for each reason type
    if reason_counts['ungrouped'] > 0:
        print(f"\n{'Ungrouped Files:'}")
        ungrouped_examples = [f for f in unmoved_files if f['reason'] == 'ungrouped'][:5]
        for f in ungrouped_examples:
            print(f"    - {f['file_path']}")
        if reason_counts['ungrouped'] > 5:
            print(f"    ... and {reason_counts['ungrouped'] - 5} more")

    if reason_counts['not_found'] > 0:
        print(f"\n{'Not Found Files:'}")
        not_found_examples = [f for f in unmoved_files if f['reason'] == 'not_found'][:5]
        for f in not_found_examples:
            unit_info = f" (Unit: {f['unit_title']})" if f['unit_title'] else ""
            print(f"    - {f['file_path']}{unit_info}")
        if reason_counts['not_found'] > 5:
            print(f"    ... and {reason_counts['not_found'] - 5} more")

    if reason_counts['failed'] > 0:
        print(f"\n{'Failed to Move:'}")
        failed_examples = [f for f in unmoved_files if f['reason'] == 'failed'][:5]
        for f in failed_examples:
            print(f"    - {f['file_path']}")
            print(f"      Reason: {f.get('error_details', 'Unknown error')}")
        if reason_counts['failed'] > 5:
            print(f"    ... and {reason_counts['failed'] - 5} more")

    print(f"\n✓ Detailed statistics saved to: {output_path}")
    print(f"{'='*60}\n")

    return statistics


def move_files_to_folders(source_dir: str | Path, base_dir: str | Path, syllabus: Dict[str, Any], dry_run: bool = False) -> Dict[str, Any]:
    """
    Move files from source directory to organized folder structure using LLM-designed paths.

    Args:
        source_dir: Source directory containing the files to organize
        base_dir: Base directory where organized folders are located
        syllabus: Syllabus JSON structure with path_mappings for each unit
        dry_run: If True, only simulate file moves without actually moving files

    Returns:
        Dictionary containing file movement log with details of all operations
    """
    import shutil

    source_dir = Path(source_dir)
    base_dir = Path(base_dir)

    print(f"\n{'[DRY RUN] ' if dry_run else ''}Moving files to LLM-designed folder structure...")
    print("=" * 60)

    move_log = {
        "operation": "file_movement",
        "mode": "dry_run" if dry_run else "execute",
        "source_directory": str(source_dir),
        "base_directory": str(base_dir),
        "course_id": syllabus.get("course_id"),
        "term": syllabus.get("term"),
        "total_files_moved": 0,
        "total_files_failed": 0,
        "total_files_not_found": 0,
        "lost_files": [],  # Track all files that couldn't be found
        "units": []
    }

    for unit in syllabus.get("units", []):
        unit_id = unit.get("unit_id", "U00")
        title = unit.get("title", "Untitled")
        folder_name = f"{title.replace(' ', '_').replace('/', '_')}"
        unit_base_folder = base_dir / folder_name

        unit_log = {
            "unit_id": unit_id,
            "folder_name": folder_name,
            "files_moved": 0,
            "files_failed": 0,
            "files_not_found": 0,
            "operations": []
        }

        # Use LLM-designed path mappings
        path_mappings = unit.get("path_mappings", [])

        if not path_mappings:
            # Fallback: use suggested_files if path_mappings not available
            print(f"  ⚠ No path mappings for {title}, using fallback (flat structure)")
            for file_rel_path in unit.get("suggested_files", []):
                path_mappings.append({
                    "source_path": file_rel_path,
                    "dest_path": Path(file_rel_path).name,
                    "subfolder": ""
                })

        for mapping in path_mappings:
            source_path = mapping.get("source_path")
            dest_rel_path = mapping.get("dest_path")

            source_file = source_dir / source_path
            # Use LLM-designed destination path
            dest_file = unit_base_folder / dest_rel_path

            operation = {
                "source": str(source_file),
                "destination": str(dest_file),
                "source_relative": source_path,
                "dest_relative": dest_rel_path,
                "status": None,
                "message": None
            }

            if not source_file.exists():
                operation["status"] = "not_found"
                operation["message"] = f"Source file does not exist"
                unit_log["files_not_found"] += 1
                move_log["total_files_not_found"] += 1

                # Add to lost files list
                move_log["lost_files"].append({
                    "file_path": source_path,
                    "unit_id": unit_id,
                    "unit_title": title
                })

                if not dry_run:
                    print(f"  ⚠ Not found: {source_path}")
            else:
                if dry_run:
                    operation["status"] = "planned"
                    operation["message"] = "Would move file"
                    unit_log["files_moved"] += 1
                    move_log["total_files_moved"] += 1
                else:
                    try:
                        # Create parent directories if needed
                        dest_file.parent.mkdir(parents=True, exist_ok=True)

                        # Copy file (use copy2 to preserve metadata)
                        shutil.copy2(source_file, dest_file)

                        operation["status"] = "success"
                        operation["message"] = "File moved successfully"
                        unit_log["files_moved"] += 1
                        move_log["total_files_moved"] += 1

                    except Exception as e:
                        operation["status"] = "failed"
                        operation["message"] = str(e)
                        unit_log["files_failed"] += 1
                        move_log["total_files_failed"] += 1
                        print(f"  ✗ Failed to move {source_path}: {e}")

            unit_log["operations"].append(operation)

        # Print unit summary
        if dry_run:
            print(f"[DRY RUN] {folder_name}: Would move {unit_log['files_moved']} files")
        else:
            print(f"✓ {folder_name}: Moved {unit_log['files_moved']} files", end="")
            if unit_log["files_not_found"] > 0:
                print(f", {unit_log['files_not_found']} not found", end="")
            if unit_log["files_failed"] > 0:
                print(f", {unit_log['files_failed']} failed", end="")
            print()

        move_log["units"].append(unit_log)

    # Print final summary
    print("\n" + "=" * 60)
    if dry_run:
        print(f"[DRY RUN] Would move {move_log['total_files_moved']} files")
    else:
        print(f"✓ Successfully moved {move_log['total_files_moved']} files")
        if move_log['total_files_not_found'] > 0:
            print(f"⚠ {move_log['total_files_not_found']} files not found")
        if move_log['total_files_failed'] > 0:
            print(f"✗ {move_log['total_files_failed']} files failed to move")

    return move_log


def create_folder_structure(base_dir: str | Path, syllabus: Dict[str, Any], dry_run: bool = False) -> Dict[str, Any]:
    """
    Create physical folder structure based on syllabus JSON.

    Args:
        base_dir: Base directory where folders will be created
        syllabus: Syllabus JSON structure
        dry_run: If True, only print what would be created without actually creating folders

    Returns:
        Dictionary containing operations log with details of all actions
    """
    base_dir = Path(base_dir)

    print(f"\n{'[DRY RUN] ' if dry_run else ''}Creating folder structure in: {base_dir}")
    print("=" * 60)

    operations_log = {
        "operation": "folder_structure_creation",
        "mode": "dry_run" if dry_run else "execute",
        "base_directory": str(base_dir),
        "course_id": syllabus.get("course_id"),
        "term": syllabus.get("term"),
        "total_units": len(syllabus.get("units", [])),
        "folders": []
    }

    for unit in syllabus.get("units", []):
        unit_id = unit.get("unit_id", "U00")
        title = unit.get("title", "Untitled")

        # Create folder name: Title (without unit_id prefix)
        folder_name = f"{title.replace(' ', '_').replace('/', '_')}"
        folder_path = base_dir / folder_name

        folder_info = {
            "unit_id": unit_id,
            "folder_name": folder_name,
            "folder_path": str(folder_path),
            "title": title,
            "description": unit.get("description", ""),
            "aliases": unit.get("aliases", []),
            "expected_types": unit.get("expected_types", []),
            "suggested_files": unit.get("suggested_files", []),
            "file_count": len(unit.get("suggested_files", [])),
            "actions": []
        }

        # Get structure design for subfolders
        structure_design = unit.get("structure_design", {})
        subfolders = structure_design.get("structure", {})
        org_type = structure_design.get("organization_type", "flat")

        if dry_run:
            print(f"[DRY RUN] Would create: {folder_path}")
            print(f"  Description: {unit.get('description', 'N/A')}")
            print(f"  Organization: {org_type}")

            folder_info["actions"].append({
                "type": "create_directory",
                "path": str(folder_path),
                "status": "planned"
            })

            # Show subfolders if designed
            if subfolders:
                print(f"  Subfolders ({len(subfolders)}):")
                for subfolder_name, subfolder_info in subfolders.items():
                    subfolder_path = folder_path / subfolder_name
                    print(f"    - {subfolder_name}: {subfolder_info.get('description', 'N/A')}")
                    folder_info["actions"].append({
                        "type": "create_subdirectory",
                        "path": str(subfolder_path),
                        "status": "planned"
                    })

            suggested_files = unit.get("suggested_files", [])
            if suggested_files:
                print(f"  Files to organize: {len(suggested_files)}")
            print()
        else:
            try:
                # Create main unit folder
                folder_path.mkdir(parents=True, exist_ok=True)
                folder_info["actions"].append({
                    "type": "create_directory",
                    "path": str(folder_path),
                    "status": "success"
                })

                print(f"✓ Created: {folder_path} ({org_type} organization)")

                # Create subfolders from LLM design
                if subfolders:
                    for subfolder_name in subfolders.keys():
                        subfolder_path = folder_path / subfolder_name
                        subfolder_path.mkdir(parents=True, exist_ok=True)
                        folder_info["actions"].append({
                            "type": "create_subdirectory",
                            "path": str(subfolder_path),
                            "status": "success"
                        })
                    print(f"  ✓ Created {len(subfolders)} subfolders")

            except Exception as e:
                folder_info["actions"].append({
                    "type": "error",
                    "message": str(e),
                    "status": "failed"
                })
                print(f"✗ Failed to create: {folder_path} - {e}")

        operations_log["folders"].append(folder_info)

    if not dry_run:
        print(f"\n✓ Folder structure created successfully!")

    return operations_log


def map_files_to_units_with_llm(files: List[Dict[str, str]], units: List[Dict[str, Any]], course_id: str, term: str, model: str = "gpt-4.1", batch_size: int = 100) -> Dict[str, Any]:
    """
    Map actual files to units using LLM with JSON schema enforcement.
    This prevents hallucination by forcing LLM to use actual file paths.
    Processes files in batches to avoid token limits.

    Args:
        files: List of actual scanned files
        units: List of units with aliases
        course_id: Course ID
        term: Term
        model: OpenAI model
        batch_size: Number of files to process per batch (default: 100)

    Returns:
        Dictionary with file mappings: {file_path: unit_id or "ungrouped"}
    """
    unit_ids = [u["unit_id"] for u in units] + ["ungrouped"]

    # Build units description once
    units_description = "\n".join([
        f"- {u['unit_id']}: {u['title']} (aliases: {', '.join(u.get('aliases', [])[:10])})"
        for u in units
    ])

    all_mappings = {}
    total_files = len(files)

    print(f"Mapping {total_files} files in batches of {batch_size}...")

    # Process files in batches
    for batch_idx in range(0, total_files, batch_size):
        batch_files = files[batch_idx:batch_idx + batch_size]
        batch_num = (batch_idx // batch_size) + 1
        total_batches = (total_files + batch_size - 1) // batch_size

        print(f"  Batch {batch_num}/{total_batches}: Processing {len(batch_files)} files...")

        # Create JSON schema for this batch
        json_schema = {
            "name": "file_mapping",
            "strict": True,
            "schema": {
                "type": "object",
                "properties": {
                    "file_mappings": {
                        "type": "array",
                        "items": {
                            "type": "object",
                            "properties": {
                                "file_path": {
                                    "type": "string",
                                    "description": "Exact file path from the provided list"
                                },
                                "unit_id": {
                                    "type": "string",
                                    "enum": unit_ids,
                                    "description": "Unit ID this file belongs to, or 'ungrouped'"
                                }
                            },
                            "required": ["file_path", "unit_id"],
                            "additionalProperties": False
                        }
                    }
                },
                "required": ["file_mappings"],
                "additionalProperties": False
            }
        }

        # Build file list for this batch
        files_list = "\n".join([f"- {f['path']}" for f in batch_files])

        prompt = f"""You are mapping course files to organizational units.

Course: {course_id} ({term})
Batch: {batch_num}/{total_batches}

Available Units:
{units_description}

Files to Organize (YOU MUST USE THESE EXACT PATHS):
{files_list}

Instructions:
1. For EACH file listed above, assign it to the most appropriate unit_id
2. Use the unit aliases to determine which unit each file belongs to
3. If a file doesn't fit any unit, assign it to "ungrouped"
4. YOU MUST map ALL {len(batch_files)} files in this batch - do not skip any
5. USE THE EXACT FILE PATHS as listed above (copy them exactly)

Return a JSON with file_mappings array where each file is mapped to a unit_id.
"""

        try:
            response = call_openai(prompt, model, use_structured_output=True, json_schema=json_schema)
            result = extract_json(response)

            # Convert to dict and add to all mappings
            batch_mapping = {}
            for item in result.get("file_mappings", []):
                batch_mapping[item["file_path"]] = item["unit_id"]

            all_mappings.update(batch_mapping)

            # Check for unmapped files in this batch
            batch_file_paths = [f["path"] for f in batch_files]
            unmapped = set(batch_file_paths) - set(batch_mapping.keys())
            if unmapped:
                print(f"  Warning: {len(unmapped)} files not mapped in batch {batch_num}, marking as ungrouped")
                for file_path in unmapped:
                    all_mappings[file_path] = "ungrouped"

            print(f"  ✓ Batch {batch_num}/{total_batches} complete: {len(batch_mapping)} files mapped")

        except Exception as e:
            print(f"  ✗ Error in batch {batch_num}: {e}")
            # Mark all files in failed batch as ungrouped
            for f in batch_files:
                all_mappings[f["path"]] = "ungrouped"

    print(f"✓ Completed mapping {len(all_mappings)} files")
    return all_mappings


def sample_files_from_unit(files: List[str], sample_size: int = 50) -> tuple[List[str], List[str]]:
    """
    Randomly sample files from a unit for ground truth generation.

    Args:
        files: All files in the unit
        sample_size: Number of files to sample (default 50)

    Returns:
        Tuple of (sampled_files, remaining_files)
    """
    import random

    if len(files) <= sample_size:
        return files, []

    # Sample files
    sampled = random.sample(files, sample_size)
    remaining = [f for f in files if f not in sampled]

    return sampled, remaining


def design_unit_structure(
    unit: Dict[str, Any],
    files: List[str],
    course_id: str,
    term: str,
    model: str = "gpt-4.1",
    use_sampling: bool = True,
    sample_size: int = 50
) -> Dict[str, Any]:
    """
    Stage 3: Design internal folder structure for a unit using SAMPLE-BASED learning.

    Analyzes a SAMPLE of files to design structure, then applies to all files.

    Args:
        unit: Unit dictionary with unit_id, title, description
        files: List of file paths belonging to this unit
        course_id: Course identifier
        term: Academic term
        model: OpenAI model to use
        use_sampling: If True, sample files for analysis (default True)
        sample_size: Number of files to sample (default 50)

    Returns:
        Dictionary with designed structure:
        {
            "unit_id": "lectures",
            "structure": {
                "Week_01": {"description": "...", "pattern": "..."},
                "Week_02": {...}
            },
            "organization_type": "chronological" | "topical" | "flat",
            "sample_size": 50,
            "total_files": 100
        }
    """

    if not files:
        return {
            "unit_id": unit["unit_id"],
            "structure": {},
            "organization_type": "flat"
        }

    # Sample files if enabled
    if use_sampling and len(files) > sample_size:
        sampled_files, _ = sample_files_from_unit(files, sample_size)
        print(f"    Sampled {len(sampled_files)} files from {len(files)} total for analysis")
        analysis_files = sampled_files
    else:
        analysis_files = files

    # Prepare file list for LLM
    file_list = "\n".join([f"- {f}" for f in analysis_files])

    total_files = len(files)
    sample_note = ""
    if use_sampling and len(files) > sample_size:
        sample_note = f"\n**NOTE**: You are analyzing a SAMPLE of {len(analysis_files)} files from {total_files} total files.\nDesign patterns that will work for ALL {total_files} files, not just this sample."

    prompt = f"""You are designing the internal folder structure for a course material category.

Course: {course_id} ({term})
Category: {unit['title']} ({unit['unit_id']})
Description: {unit.get('description', 'N/A')}
Expected Types: {', '.join(unit.get('expected_types', []))}
Total Files: {len(files)}

Files in this category:
{file_list}

### Task
Analyze these files and design a folder hierarchy that **groups files with CLEAR, OBVIOUS relationships**.

**KEY PRINCIPLE**: Only group files together when there is STRONG EVIDENCE they are related. Be STRICT and CONSERVATIVE.

### Instructions

1. **Identify Clear Patterns ONLY**:
   - Look for EXACT matches in dates, numbers, or topics
   - Example: "10-10 ARUCO.pdf" and "10-10 ARUCO.mp4" → SAME date → group together
   - Example: "Lecture_01.pdf" and "Lecture_01.mp4" → SAME number → group together
   - **DO NOT guess or imagine relationships** - only group with clear evidence

2. **Identify Relationships (Be STRICT)**:
   - **Same exact date**: Files with identical dates (e.g., both "10-10") go together
   - **Same exact number**: Files with same number (e.g., both "01" or "hw1") go together
   - **Same exact topic keyword**: Files with identical topic words (case-insensitive)
   - **Multiple file types**: Only group if they have matching dates/numbers/topics
   - **When in doubt, keep separate** - better to have more folders than wrong groupings

3. **Choose Organization Type**:
   - **chronological**: Organize by lecture/session number, date, or sequence
     - Good for: Lectures (Lecture 1, 2, 3), Discussions, Labs
     - Group by week, lecture number, or date range
   - **topical**: Organize by subject/theme
     - Good for: Topics that span multiple sessions (ARUCO, Perception, Audio)
   - **hybrid**: Chronological with topic names (Week_06_ARUCO)
   - **flat**: No subdirectories (few files or unclear grouping)

4. **Design Subdirectories**:
   - Create subfolders that contain files with MATCHING patterns
   - Example: "Lecture_01/" contains files with "01" or specific date
   - Use clear naming with numbering: "Lecture_01", "Lecture_02", "Lecture_03"
   - Aim for 5-15 subdirectories
   - Each subfolder has a clear matching criteria (date, number, or exact keyword)

5. **Pattern Matching (Strict)**:
   - Patterns must PRECISELY match files that belong together
   - Use exact dates: "10-10" matches only "10-10", not "10-15"
   - Use exact numbers: "01" matches "01", not "02"
   - Use exact keywords: "ARUCO" (case-insensitive)
   - Example good pattern: "10-10" for files on Oct 10
   - Example good pattern: "Lecture.01|01.*Lecture" for lecture 1

6. **Output Requirements**: Return JSON with this structure:
{{
  "unit_id": "{unit['unit_id']}",
  "organization_type": "chronological|topical|hybrid|flat",
  "structure": {{
    "folder_name": {{
      "description": "What belongs here",
      "pattern": "Regex or keyword pattern to match files",
      "order": 1
    }}
  }},
  "reasoning": "Brief explanation of design choice"
}}

### Examples

**Example 1: Strict Date-Based Grouping**
{{
  "organization_type": "chronological",
  "structure": {{
    "Lecture_8-29": {{
      "description": "Files from 8-29 (lecture slides and videos with this exact date)",
      "pattern": "8-29",
      "order": 1
    }},
    "Lecture_9-05": {{
      "description": "Files from 9-05 (lecture materials with this exact date)",
      "pattern": "9-05",
      "order": 2
    }},
    "Lecture_10-10": {{
      "description": "Files from 10-10 (ARUCO lecture materials with this exact date)",
      "pattern": "10-10",
      "order": 6
    }}
  }},
  "reasoning": "Each folder contains files with the exact same date, ensuring related materials (slides + videos) from same lecture day are grouped"
}}

**Example 2: Strict Number-Based Grouping**
{{
  "organization_type": "chronological",
  "structure": {{
    "Homework_01": {{
      "description": "Homework 1 materials (all files with '01' or 'hw1')",
      "pattern": "hw.?01|01.*hw|homework.?01",
      "order": 1
    }},
    "Homework_02": {{
      "description": "Homework 2 materials",
      "pattern": "hw.?02|02.*hw|homework.?02",
      "order": 2
    }}
  }},
  "reasoning": "Each folder groups files with exact matching homework numbers"
}}

**Example 3: Strict Keyword-Based Grouping**
{{
  "organization_type": "topical",
  "structure": {{
    "ARUCO_Materials": {{
      "description": "All files with 'ARUCO' or 'aruco' in the name",
      "pattern": "[Aa][Rr][Uu][Cc][Oo]",
      "order": 1
    }},
    "Perception_Materials": {{
      "description": "All files with 'perception' in the name",
      "pattern": "[Pp]erception",
      "order": 2
    }}
  }},
  "reasoning": "Each folder groups files with exact matching topic keywords (case-insensitive)"
}}

Output ONLY valid JSON, no additional text.
"""

    try:
        response = call_openai(prompt, model, use_structured_output=False)
        structure_design = extract_json(response)
        print(f"  ✓ Designed structure for {unit['title']}: {structure_design.get('organization_type', 'unknown')}")
        return structure_design
    except Exception as e:
        print(f"  ✗ Failed to design structure for {unit['title']}: {e}")
        return {
            "unit_id": unit["unit_id"],
            "structure": {},
            "organization_type": "flat",
            "reasoning": f"Error: {e}"
        }


def generate_path_mappings(
    unit: Dict[str, Any],
    structure_design: Dict[str, Any],
    files: List[str],
    model: str = "gpt-4.1",
    batch_size: int = 50
) -> List[Dict[str, str]]:
    """
    Stage 4: Generate exact source→destination path mappings.

    For each file, determine its exact destination path within the designed structure.

    Args:
        unit: Unit dictionary
        structure_design: Structure design from design_unit_structure()
        files: List of source file paths
        model: OpenAI model to use
        batch_size: Number of files to process per LLM call

    Returns:
        List of mappings:
        [
            {
                "source_path": "Bcourses/Lecture_Videos/10-10 ARUCO.mp4",
                "dest_path": "Week_06/10-10_ARUCO_Markers.mp4",
                "subfolder": "Week_06"
            }
        ]
    """

    if not files:
        return []

    all_mappings = []
    total_files = len(files)

    # Get available subfolders
    subfolders = list(structure_design.get("structure", {}).keys())
    if not subfolders:
        subfolders = [""]  # Flat structure

    print(f"  Mapping {total_files} files to structure with {len(subfolders)} subfolders...")

    # Process in batches
    for batch_idx in range(0, total_files, batch_size):
        batch_files = files[batch_idx:batch_idx + batch_size]
        batch_num = (batch_idx // batch_size) + 1
        total_batches = (total_files + batch_size - 1) // batch_size

        # Prepare structure description
        structure_desc = ""
        for folder, info in structure_design.get("structure", {}).items():
            structure_desc += f"- {folder}: {info.get('description', 'N/A')} (pattern: {info.get('pattern', 'any')})\n"

        if not structure_desc:
            structure_desc = "- (root): All files go directly in unit folder\n"

        file_list = "\n".join([f"- {f}" for f in batch_files])

        prompt = f"""You are mapping files to their destination paths using the RELATIVE PATH as evidence.

Unit: {unit['title']} ({unit['unit_id']})
Organization Type: {structure_design.get('organization_type', 'flat')}
Batch: {batch_num}/{total_batches}

Available Subfolders:
{structure_desc}

Files to Map (USE EXACT PATHS):
{file_list}

### Task
For each file, analyze its RELATIVE PATH to determine the best subfolder.

**CRITICAL RULES**:
1. **Analyze the full relative path** (e.g., "Bcourses/Lecture_Slides_2024/10-10 ARUCO.pdf")
2. **Use path information as evidence** (dates, numbers, keywords in path)
3. **Patterns are HINTS, not strict rules** - they guide but don't dictate
4. **DO NOT rename files** - keep original filename exactly as-is
5. **Be conservative** - only group when there's clear evidence in the path

### Guidelines - Path-Based Grouping

1. **Analyze the Relative Path**:
   - Full path: "Bcourses/Lecture_Slides_2024/10-10 ARUCO Markers.pdf"
   - Extract date: "10-10"
   - Extract topic: "ARUCO Markers"
   - Source folder: "Lecture_Slides_2024" (indicates lecture content)

2. **Match to Subfolders Using Path Evidence**:
   - Look for dates in path: "10-10" in filename → matches "Lecture_10-10" subfolder
   - Look for numbers: "01" or "hw1" in path → matches "Homework_01" subfolder
   - Look for keywords: "ARUCO" in path → matches "ARUCO_Materials" subfolder
   - Use patterns as HINTS, not strict rules - it's okay to match "10-10" with "Lecture_Oct10"

3. **Group Related Files Conservatively**:
   - Files with SAME date → likely same folder (e.g., both "10-10")
   - Files with SAME number → likely same folder (e.g., both "hw01")
   - Files with SAME topic keyword → likely same folder (e.g., both "ARUCO")
   - **BUT**: Only group when there's CLEAR evidence in the path
   - Don't guess or imagine relationships - use what's in the path

4. **Keep Original Filenames**:
   - **NEVER rename files**
   - dest_path = subfolder + "/" + original_filename
   - Example: "10-10 ARUCO Markers.pdf" → "Lecture_10-10/10-10 ARUCO Markers.pdf"
   - Preserve spaces, capitalization, everything exactly

5. **Folder Assignment Strategy**:
   - Check if path contains subfolder pattern/keywords
   - If clear match → assign to that subfolder
   - If multiple matches → pick the most specific/relevant one
   - If no clear match → use root folder ("")

### Examples

**Example 1: Date-Based Grouping (Same Date → Same Folder)**

Available Subfolders:
- Lecture_10-10: pattern "10-10"
- Lecture_10-15: pattern "10-15"

Files:
- "Bcourses/Lecture_Slides_2024/10-10 ARUCO Markers.pdf"
- "Bcourses/Lecture_Videos_2024/10-10 ARUCO Markers.mp4"

Correct Mapping (grouped by same date):
[
  {{
    "source_path": "Bcourses/Lecture_Slides_2024/10-10 ARUCO Markers.pdf",
    "dest_path": "Lecture_10-10/10-10 ARUCO Markers.pdf",
    "subfolder": "Lecture_10-10"
  }},
  {{
    "source_path": "Bcourses/Lecture_Videos_2024/10-10 ARUCO Markers.mp4",
    "dest_path": "Lecture_10-10/10-10 ARUCO Markers.mp4",
    "subfolder": "Lecture_10-10"
  }}
]

→ Both have "10-10" in path → same folder, filenames unchanged

**Example 2: Different Dates → Different Folders**

Files:
- "Bcourses/Lecture_Slides_2024/10-10 ARUCO.pdf"
- "Bcourses/Lecture_Videos_2024/10-15 ARUCO.mp4"

Correct Mapping (different dates):
[
  {{
    "source_path": "Bcourses/Lecture_Slides_2024/10-10 ARUCO.pdf",
    "dest_path": "Lecture_10-10/10-10 ARUCO.pdf",
    "subfolder": "Lecture_10-10"
  }},
  {{
    "source_path": "Bcourses/Lecture_Videos_2024/10-15 ARUCO.mp4",
    "dest_path": "Lecture_10-15/10-15 ARUCO.mp4",
    "subfolder": "Lecture_10-15"
  }}
]

→ Different dates in path → different folders, filenames unchanged

### Output Format
Return JSON array:
[
  {{
    "source_path": "exact/source/path.ext",
    "dest_path": "subfolder/renamed_file.ext",
    "subfolder": "subfolder_name"
  }}
]

If flat structure (no subfolders), use:
"dest_path": "renamed_file.ext",
"subfolder": ""

Map ALL {len(batch_files)} files in this batch.
Output ONLY valid JSON array.
"""

        try:
            # Create JSON schema for structured output
            json_schema = {
                "name": "path_mappings",
                "strict": True,
                "schema": {
                    "type": "object",
                    "properties": {
                        "mappings": {
                            "type": "array",
                            "items": {
                                "type": "object",
                                "properties": {
                                    "source_path": {"type": "string"},
                                    "dest_path": {"type": "string"},
                                    "subfolder": {"type": "string"}
                                },
                                "required": ["source_path", "dest_path", "subfolder"],
                                "additionalProperties": False
                            }
                        }
                    },
                    "required": ["mappings"],
                    "additionalProperties": False
                }
            }

            response = call_openai(prompt, model, use_structured_output=True, json_schema=json_schema)
            result = extract_json(response)
            batch_mappings = result.get("mappings", [])

            all_mappings.extend(batch_mappings)
            print(f"    ✓ Batch {batch_num}/{total_batches}: Mapped {len(batch_mappings)} files")

        except Exception as e:
            print(f"    ✗ Batch {batch_num}/{total_batches} failed: {e}")
            # Fallback: flat mapping with original filenames
            for file_path in batch_files:
                all_mappings.append({
                    "source_path": file_path,
                    "dest_path": Path(file_path).name,
                    "subfolder": ""
                })

    return all_mappings


def enhance_existing_syllabus(
    syllabus_path: str | Path,
    output_path: str | Path | None = None,
    model: str = "gpt-4.1"
) -> Dict[str, Any]:
    """
    Enhance an existing syllabus.json with Stage 3 & 4 (structure design + path mappings).

    Use this to upgrade old syllabus files without re-running Stage 1 & 2.

    Args:
        syllabus_path: Path to existing syllabus.json
        output_path: Output path for enhanced syllabus (None = overwrite original)
        model: OpenAI model to use

    Returns:
        Enhanced syllabus dictionary with structure_design and path_mappings

    Example:
        syllabus = enhance_existing_syllabus("syllabus.json", "enhanced_syllabus.json")
    """
    syllabus_path = Path(syllabus_path)
    if not syllabus_path.exists():
        raise FileNotFoundError(f"Syllabus file not found: {syllabus_path}")

    # Load existing syllabus
    with open(syllabus_path, "r", encoding="utf-8") as f:
        syllabus = json.load(f)

    print(f"Enhancing existing syllabus: {syllabus_path}")
    print(f"Course: {syllabus.get('course_id')} ({syllabus.get('term')})")
    print(f"Units: {len(syllabus.get('units', []))}")

    course_id = syllabus.get("course_id", "Unknown")
    term = syllabus.get("term", "Unknown")

    # Stage 3: Design internal structure for each unit
    print(f"\nStage 3: Designing internal folder structure for each unit...")
    for unit in syllabus["units"]:
        unit_files = unit.get("suggested_files", [])
        if unit_files:
            structure_design = design_unit_structure(
                unit=unit,
                files=unit_files,
                course_id=course_id,
                term=term,
                model=model
            )
            unit["structure_design"] = structure_design
        else:
            unit["structure_design"] = {
                "unit_id": unit["unit_id"],
                "structure": {},
                "organization_type": "flat"
            }

    # Stage 4: Generate path mappings for each unit
    print(f"\nStage 4: Generating destination path mappings for each unit...")
    for unit in syllabus["units"]:
        unit_files = unit.get("suggested_files", [])
        structure_design = unit.get("structure_design", {})

        if unit_files:
            path_mappings = generate_path_mappings(
                unit=unit,
                structure_design=structure_design,
                files=unit_files,
                model=model,
                batch_size=50
            )
            unit["path_mappings"] = path_mappings
        else:
            unit["path_mappings"] = []

    # Save enhanced syllabus
    if output_path is None:
        output_path = syllabus_path
    else:
        output_path = Path(output_path)

    output_path.parent.mkdir(parents=True, exist_ok=True)

    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(syllabus, f, ensure_ascii=False, indent=2)

    print(f"\n✓ Enhanced syllabus saved to: {output_path}")

    return syllabus


def generate_folder_structure(
    scan_dir: str | Path,
    course_id: str,
    term: str,
    output: str | Path = "syllabus.json",
    model: str = "gpt-4.1",
    create_folders: bool = False,
    output_dir: str | Path | None = None,
    dry_run: bool = False,
    move_files: bool = False,
    operations_log: str | Path | None = None,
    move_log: str | Path | None = None,
    ignore_file: str | Path | None = None,
    file_statistics: str | Path | None = None,
) -> Dict[str, Any]:
    """
    Generate a folder structure for course materials using LLM analysis.

    Args:
        scan_dir: Directory containing course files to analyze
        course_id: Course identifier (e.g., "CS61A", "EE106B")
        term: Academic term (e.g., "2025FA", "2024SP")
        output: Output path for syllabus.json (default: "syllabus.json")
        model: OpenAI model to use (default: "gpt-4.1")
        create_folders: Whether to create physical folder structure (default: False)
        output_dir: Directory where folders will be created (required if create_folders=True)
        dry_run: Preview without making changes (default: False)
        move_files: Move files to organized folders (requires create_folders=True)
        operations_log: Custom path for operations log (None = auto)
        move_log: Custom path for move log (None = auto)
        ignore_file: Path to ignore patterns file (None = auto-detect .scanignore)
        file_statistics: Custom path for file statistics report (None = auto)

    Returns:
        Dictionary containing the generated syllabus structure

    Raises:
        ValueError: If validation fails
        FileNotFoundError: If scan_dir doesn't exist
        RuntimeError: If OpenAI API call fails
    """
    # Validation
    if create_folders and not output_dir:
        raise ValueError("output_dir is required when create_folders is True")
    if move_files and not create_folders:
        raise ValueError("move_files requires create_folders to be True")

    # Step 1: Scan directory
    print(f"Scanning directory: {scan_dir}")
    files, ignored_count = scan_directory(scan_dir, ignore_file=ignore_file)
    total_files = len(files)  # This is the count AFTER ignoring

    if ignored_count > 0:
        print(f"Found {total_files} files to process (ignored {ignored_count} additional files)")
    else:
        print(f"Found {total_files} files to process")

    if total_files == 0:
        raise ValueError("No files found in directory (after applying ignore patterns)")

    # Step 2: Generate units (categories) with LLM
    print(f"\nStep 1: Analyzing {total_files} files to identify categories with {model}...")
    file_list_text = format_file_list(files)
    prompt = FOLDER_STRUCTURE_PROMPT.format(
        course_id=course_id,
        term=term,
        file_count=total_files,
        file_list=file_list_text
    )

    response = call_openai(prompt, model)

    # Step 3: Extract and validate JSON
    print("Extracting units from response...")
    syllabus = extract_json(response)

    # Validate structure
    if "units" not in syllabus or not isinstance(syllabus["units"], list):
        raise ValueError("Invalid syllabus structure: missing 'units' array")

    print(f"✓ Generated {len(syllabus['units'])} units")

    # Step 4: Map actual files to units using JSON schema enforcement
    print(f"\nStep 2: Mapping {total_files} files to units with JSON schema...")
    file_mapping = map_files_to_units_with_llm(files, syllabus["units"], course_id, term, model)

    # Step 5: Populate suggested_files for each unit based on mapping
    for unit in syllabus["units"]:
        unit_id = unit["unit_id"]
        unit["suggested_files"] = sorted([
            file_path for file_path, mapped_unit in file_mapping.items()
            if mapped_unit == unit_id
        ])

    # Step 6: Collect ungrouped files
    syllabus["ungrouped_files"] = sorted([
        file_path for file_path, mapped_unit in file_mapping.items()
        if mapped_unit == "ungrouped"
    ])

    # Validate file coverage
    total_grouped_files = sum(len(unit.get('suggested_files', [])) for unit in syllabus.get('units', []))
    ungrouped_count = len(syllabus.get('ungrouped_files', []))
    coverage_percent = (total_grouped_files / total_files) * 100 if total_files > 0 else 0

    print(f"\n✓ File mapping complete:")
    print(f"  - Grouped: {total_grouped_files}/{total_files} ({coverage_percent:.1f}%)")
    print(f"  - Ungrouped: {ungrouped_count}")

    if coverage_percent < 80:
        print(f"\n⚠️ WARNING: Only {coverage_percent:.1f}% of files are grouped!")
        print(f"   Consider reviewing unit aliases or adjusting the categories.")

    # NEW Stage 3: Design internal structure for each unit
    print(f"\nStep 3: Designing internal folder structure for each unit...")
    for unit in syllabus["units"]:
        unit_files = unit.get("suggested_files", [])
        if unit_files:
            structure_design = design_unit_structure(
                unit=unit,
                files=unit_files,
                course_id=course_id,
                term=term,
                model=model
            )
            unit["structure_design"] = structure_design
        else:
            unit["structure_design"] = {
                "unit_id": unit["unit_id"],
                "structure": {},
                "organization_type": "flat"
            }

    # NEW Stage 4: Generate path mappings for each unit
    print(f"\nStep 4: Generating destination path mappings for each unit...")
    for unit in syllabus["units"]:
        unit_files = unit.get("suggested_files", [])
        structure_design = unit.get("structure_design", {})

        if unit_files:
            path_mappings = generate_path_mappings(
                unit=unit,
                structure_design=structure_design,
                files=unit_files,
                model=model,
                batch_size=50
            )
            unit["path_mappings"] = path_mappings
        else:
            unit["path_mappings"] = []

    print(f"\n✓ Structure design and path mapping complete!")

    # Step 7: Write syllabus JSON
    output_path = Path(output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(syllabus, f, ensure_ascii=False, indent=2)

    print(f"✓ Wrote syllabus to: {output_path}")

    # Step 8: Create folders if requested
    ops_log = None
    if create_folders:
        ops_log = create_folder_structure(
            base_dir=output_dir,
            syllabus=syllabus,
            dry_run=dry_run
        )

        # Save operations log
        if operations_log:
            log_path = Path(operations_log)
        else:
            # Default: save in output directory or same as syllabus
            if output_dir:
                log_path = Path(output_dir) / "operations_log.json"
            else:
                log_path = output_path.parent / "operations_log.json"

        log_path.parent.mkdir(parents=True, exist_ok=True)

        with open(log_path, "w", encoding="utf-8") as f:
            json.dump(ops_log, f, ensure_ascii=False, indent=2)

        print(f"✓ Wrote operations log to: {log_path}")

    # Step 9: Move files if requested
    if move_files:
        mv_log = move_files_to_folders(
            source_dir=scan_dir,
            base_dir=output_dir,
            syllabus=syllabus,
            dry_run=dry_run
        )

        # Save move log
        if move_log:
            move_log_path = Path(move_log)
        else:
            if output_dir:
                move_log_path = Path(output_dir) / "move_log.json"
            else:
                move_log_path = output_path.parent / "move_log.json"

        move_log_path.parent.mkdir(parents=True, exist_ok=True)

        with open(move_log_path, "w", encoding="utf-8") as f:
            json.dump(mv_log, f, ensure_ascii=False, indent=2)

        print(f"✓ Wrote move log to: {move_log_path}")

    # Step 10: Generate file statistics report
    if file_statistics or move_files or create_folders:
        # Automatically generate statistics if folders were created or files were moved
        if file_statistics:
            stats_path = Path(file_statistics)
        else:
            if output_dir:
                stats_path = Path(output_dir) / "file_statistics.json"
            else:
                stats_path = output_path.parent / "file_statistics.json"

        stats_path.parent.mkdir(parents=True, exist_ok=True)

        record_file_statistics(
            syllabus=syllabus,
            move_log=mv_log if move_files else None,
            output_path=stats_path
        )

    # Step 11: Print summary
    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    print(f"Course: {syllabus.get('course_id')} ({syllabus.get('term')})")
    print(f"Units: {len(syllabus.get('units', []))}")

    # File statistics (all counts are AFTER applying ignore patterns)
    total_grouped_files = sum(len(unit.get('suggested_files', [])) for unit in syllabus.get('units', []))
    ungrouped_files = syllabus.get('ungrouped_files', [])
    ungrouped_count = len(ungrouped_files)

    print(f"\nFile Statistics (after applying ignore patterns):")
    print(f"  Total files to organize: {total_files}")
    if ignored_count > 0:
        print(f"  Files ignored by patterns: {ignored_count}")
    print(f"  Grouped into units: {total_grouped_files}")
    print(f"  Ungrouped: {ungrouped_count}")

    # Calculate accounting
    accounted_for = total_grouped_files + ungrouped_count
    if accounted_for != total_files:
        discrepancy = total_files - accounted_for
        print(f"  ⚠️  Discrepancy: {discrepancy} files (LLM may have missed some files)")

    print("\nUnits created:")
    for i, unit in enumerate(syllabus.get("units", []), 1):
        print(f"  {i}. {unit.get('unit_id')}: {unit.get('title')}")
        print(f"     Files: {len(unit.get('suggested_files', []))}")

    if ungrouped_files:
        print(f"\n⚠ {ungrouped_count} files were not grouped:")
        for f in ungrouped_files[:10]:  # Show first 10
            print(f"  - {f}")
        if ungrouped_count > 10:
            print(f"  ... and {ungrouped_count - 10} more")
        print("\nConsider reviewing these files and updating the syllabus if needed.")

    return syllabus


def main():
    """
    Main entry point for command-line usage.
    For library usage, call generate_folder_structure() directly.
    """
    # Example configuration - modify these values or load from a config file
    config = {
        # Required settings
        "scan_dir": "/home/bot/bot/yk/YK_final/courses/CS 61A",
        "course_id": "CS 61A",
        "term": "2025SM",
        "output": "61A_syllabus.json",

        # Optional settings
        "model": "gpt-4.1",
        "create_folders": True,
        "output_dir": "/home/bot/bot/yk/YK_final/test_folder/CS 61A",
        "dry_run": False,
        "move_files": True,
        "operations_log": None,
        "move_log": None,
        "ignore_file": None,  # None = auto-detect .scanignore in scan_dir
        "file_statistics": None,  # None = auto-generate in output_dir
    }

    try:
        generate_folder_structure(**config)
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
