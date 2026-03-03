#!/usr/bin/env python3
"""
File Rearrangement Specification Generator

This module analyzes a course directory and generates a comprehensive markdown
specification document that defines how files should be organized and rearranged.

The LLM analyzes actual file samples to recommend:
- File categories and groups
- Naming conventions and patterns
- Organization rules
- Special handling for metadata, solutions, etc.

Usage:
    python spec_generator.py

Or as a library:
    from spec_generator import generate_specification

    spec = generate_specification(
        scan_dir="/path/to/CS 61A",
        course_id="CS 61A",
        term="2025SM",
        output="CS61A_rearrangement_spec.md"
    )
"""

import json
import os
import random
from pathlib import Path
from typing import Dict, List, Any
from collections import Counter
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

def sample_files_from_directory(directory: Path, sample_size: int = 100, ignore_file: Path | None = None) -> Dict[str, Any]:
    """
    Sample representative files from a directory for analysis.

    Args:
        directory: Directory to sample from
        sample_size: Target number of files to sample
        ignore_file: Path to .scanignore file (None = auto-detect in directory)

    Returns:
        Dictionary with sampled files and directory statistics
    """
    from pathlib import Path
    import fnmatch

    directory = Path(directory)

    # Load ignore patterns from .scanignore
    if ignore_file is None:
        default_ignore = directory / '.scanignore'
        if default_ignore.exists():
            ignore_file = default_ignore

    ignore_patterns = set()
    if ignore_file and Path(ignore_file).exists():
        with open(ignore_file, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith('#'):
                    ignore_patterns.add(line)
        print(f"Loaded {len(ignore_patterns)} ignore patterns from {ignore_file}")

    # Helper function to check if path should be ignored
    def should_ignore(rel_path_str: str, filename: str) -> bool:
        for pattern in ignore_patterns:
            # Check filename
            if fnmatch.fnmatch(filename, pattern):
                return True
            # Check relative path
            if fnmatch.fnmatch(rel_path_str, pattern):
                return True
        return False

    # Collect all files
    all_files = []
    ignored_count = 0

    for file_path in directory.rglob("*"):
        if file_path.is_file() and not file_path.name.startswith('.'):
            rel_path = file_path.relative_to(directory)
            rel_path_str = str(rel_path)

            # Check ignore patterns
            if should_ignore(rel_path_str, file_path.name):
                ignored_count += 1
                continue

            all_files.append({
                "path": rel_path_str,
                "name": file_path.name,
                "extension": file_path.suffix,
                "size_kb": file_path.stat().st_size // 1024,
                "parent_dir": str(rel_path.parent)
            })

    if ignored_count > 0:
        print(f"Ignored {ignored_count} files based on .scanignore patterns")

    # Directory structure analysis
    top_level_dirs = set()
    extension_counts = Counter()
    dir_file_counts = Counter()

    for f in all_files:
        parts = Path(f["path"]).parts
        if len(parts) > 0:
            top_level_dirs.add(parts[0])
            dir_file_counts[parts[0]] += 1
        extension_counts[f["extension"]] += 1

    # Sample strategy: stratified sampling by top-level directory
    sampled_files = []
    files_by_dir = {}

    for f in all_files:
        parts = Path(f["path"]).parts
        if len(parts) > 0:
            top_dir = parts[0]
            if top_dir not in files_by_dir:
                files_by_dir[top_dir] = []
            files_by_dir[top_dir].append(f)

    # Sample proportionally from each directory
    total_files = len(all_files)
    for top_dir, files in files_by_dir.items():
        proportion = len(files) / total_files
        dir_sample_size = max(5, int(sample_size * proportion))  # At least 5 per directory
        dir_sample_size = min(dir_sample_size, len(files))  # Don't exceed available files

        sampled = random.sample(files, dir_sample_size)
        sampled_files.extend(sampled)

    # If we have too many, randomly reduce
    if len(sampled_files) > sample_size:
        sampled_files = random.sample(sampled_files, sample_size)

    return {
        "total_files": total_files,
        "sampled_count": len(sampled_files),
        "sampled_files": sorted(sampled_files, key=lambda x: x["path"]),
        "statistics": {
            "top_level_directories": sorted(list(top_level_dirs)),
            "file_counts_by_directory": dict(dir_file_counts.most_common()),
            "extension_counts": dict(extension_counts.most_common()),
            "total_directories": len(top_level_dirs)
        }
    }


def call_openai_for_spec(
    sampled_data: Dict[str, Any],
    course_id: str,
    term: str,
    model: str = "gpt-4.1"
) -> str:
    """
    Call OpenAI API to analyze files and generate specification.

    Args:
        sampled_data: Dictionary with sampled files and statistics
        course_id: Course identifier
        term: Academic term
        model: OpenAI model to use

    Returns:
        Markdown specification document as string
    """
    from openai import OpenAI

    api_key = os.getenv("OPENAI_API_KEY")
    if not api_key:
        raise RuntimeError("OPENAI_API_KEY not set in environment")

    # Prepare file list for LLM
    file_list = "\n".join([
        f"- {f['path']} ({f['extension']}, {f['size_kb']}KB)"
        for f in sampled_data["sampled_files"]
    ])

    # Prepare statistics
    stats = sampled_data["statistics"]
    stats_text = f"""
**Total Files in Course**: {sampled_data['total_files']}
**Sampled Files for Analysis**: {sampled_data['sampled_count']}

**Top-Level Directories**: {', '.join(stats['top_level_directories'])}

**File Distribution by Directory**:
{chr(10).join([f'  - {dir}: {count} files' for dir, count in list(stats['file_counts_by_directory'].items())[:15]])}

**File Types**:
{chr(10).join([f'  - {ext or "(no extension)"}: {count} files' for ext, count in list(stats['extension_counts'].items())[:15]])}
"""

    prompt = f"""You are an expert course material organizer tasked with REDESIGNING the file structure for {course_id} ({term}).

## Critical Context

The current file structure is messy and inconsistent. Your job is to:
- **PROPOSE A CLEAN, STANDARDIZED REORGANIZATION** that works across different courses
- **DO NOT simply mirror the existing directory structure** - that defeats the purpose
- **CREATE UNIVERSAL CATEGORIES** that would work for any university course
- **STANDARDIZE NAMING** even if current files use different conventions

## Your Task

Analyze the current messy structure and design a **clean, standardized organization system** with:

1. **Universal Categories** - Standard groups that work for ANY course (e.g., "Assignments", "Lectures", "Exams")
2. **Rearrangement Rules** - How to map messy current files to clean new structure
3. **Pattern Recognition** - Identify files by content/purpose, not just current location
4. **Standardization** - Use consistent naming across different file types

## Course Statistics

{stats_text}

## Sampled Files (Current Messy Structure)

{file_list}

## Key Principles

**THINK CROSS-COURSE**: Your specification should work for ANY university course, not just this one.
- Stanford CS229 should use the same categories as Berkeley CS61A
- Same structure for intro courses and advanced courses
- Handle different naming conventions: "hw01", "homework_1", "HW-1", "assignment01"

**PROPOSE TRUE REARRANGEMENT**: Don't just say "keep discussions in discussions/". Instead:
- Identify what makes something a "discussion" (worksheet, number, solutions)
- Map from messy current paths → clean new structure
- Example: Both "disc/disc06/" AND "discussion_worksheets/week6/" → "Discussions/Week_06/"

**HANDLE VARIATIONS**: Courses might have:
- Different naming: "hw", "homework", "assignment", "problem_set"
- Different structures: flat vs nested directories
- Different file types: PDFs, HTML, Jupyter notebooks, videos
- Your rules should handle ALL variations

## Required Specification Sections

### 1. Universal Category System
Define 6-10 universal categories that work for this course:
- **Category Name** (standardized, like "Assignments" not "hw")
- **Purpose**: What belongs here
- **File Indicators**: Keywords, patterns, file types that identify this category
- **Handle Variations**: List all naming variations (hw, homework, assignment, etc.)

Example:
**Category: Assignments**
- Purpose: Homework, problem sets, take-home exercises
- Indicators: Keywords ["hw", "homework", "assignment", "problemset", "ps", "pset"]
- Variations: "hw01", "homework_1", "assignment-01", "ps1", "HW_1"
- Location patterns: In directories matching ["hw", "homework", "assignments", "problemsets"]

### 2. Rearrangement Mapping Rules

For EACH category, provide:
- **Detection Rules**: How to identify files that belong to this category
- **Destination Pattern**: Where files should be placed in the new structure
- **Filename Standardization**: How to rename files consistently
- **Subcategory Rules**: How to organize within the category (solutions, metadata, etc.)

**Format:**

**Category: [Name]**
- **Detection**: Keywords, patterns, file extensions to match
- **Destination**: `Category/{{identifier}}_Subcategory/`
- **Standardization**: Filename pattern (e.g., `category_{{number}}.{{ext}}`)
- **Special Handling**: Solutions, metadata, archives, videos
- **Number Extraction**: How to extract numbering from filenames/paths

### 3. Video Relationship Handling (CRITICAL)

**KEY PRINCIPLE**: Videos should be placed WITH their related content, not in a separate Videos folder.

Analyze video folder/file names to detect relationships:

**Video Type Detection:**
- **Exam Walkthroughs**: Contains exam keywords (midterm, final, mt1, mt2) + year/term
  - Example: `"CS 61A Fall 2017 Midterm 1/"`, `"[CS 61A SP25] Midterm 2 Walkthrough"`
  - Destination: `Exams/{{Year}}_{{Term}}_{{ExamType}}/videos/`

- **Lecture Topic Videos**: Folder name is a topic/concept (not an event)
  - Example: `"Sequences and Containers (Su25)"`, `"Higher-Order Functions"`, `"Tail Calls (Su25)"`
  - Destination: `Lectures/{{Topic}}/videos/`

- **Discussion Videos**: Contains "Discussion" + number or "Discussion {{N}}"
  - Example: `"Discussion 5： Iterators, Generators"`, `"[CS 61A SU25] Discussion 3"`
  - Destination: `Discussions/{{NN}}_Discussion/videos/`

- **Standalone Videos**: Generic tutorials without clear relationship
  - Destination: `Videos/` (fallback only)

**Relationship Extraction:**
- Extract year/term from video name: "Fall 2017" → 2017_Fall
- Extract exam type: "Midterm 1", "Final", "mt2" → Midterm_1, Final, Midterm_2
- Extract topic: "Sequences and Containers" → Sequences_and_Containers
- Extract discussion number: "Discussion 5" → 05

**Mapping Rules:**
- Exam videos → `Exams/{{Year}}_{{Term}}_{{ExamType}}/videos/{{filename}}`
- Lecture videos → `Lectures/{{Topic}}/videos/{{filename}}`
- Discussion videos → `Discussions/{{NN}}_Discussion/videos/{{filename}}`
- Standalone videos → `Videos/{{filename}}`

### 4. File Type Handling
- **Primary files**: Main content (.pdf, .html, .ipynb)
- **Metadata files**: .yaml, .json - keep with primary file in metadata/ subfolder
- **Archives**: .zip - keep as-is (starter code)
- **Solutions**: Always in solutions/ subfolder
- **Videos**: Place WITH related content (see Video Relationship Handling above)

### 5. Proposed Clean Structure
Show the FINAL standardized structure (not current):
```
{{course_id}}/
├── Assignments/
│   ├── 01_Homework/
│   │   ├── homework_01.zip
│   │   ├── homework_01_description.html
│   │   └── metadata/
│   ├── 02_Homework/
│   └── solutions/
├── Lectures/
│   ├── 01_Introduction/
│   │   ├── lecture_notes.html
│   │   ├── slides/
│   │   │   └── slides.pdf
│   │   └── videos/
│   │       └── lecture_video.mp4
│   ├── Sequences_and_Containers/
│   │   └── videos/
│   │       └── 02_Lists.webm
│   └── ...
├── Discussions/
│   ├── 05_Discussion/
│   │   ├── worksheet.pdf
│   │   └── videos/
│   │       └── problem1.mkv
│   └── solutions/
├── Labs/
├── Projects/
├── Exams/
│   ├── 2017_Fall_Midterm_1/
│   │   ├── exam.pdf
│   │   ├── videos/
│   │   │   └── 05_Question_4b.mkv
│   │   └── solutions/
│   ├── 2025_Spring_Final/
│   └── ...
├── Resources/
│   ├── syllabus.html
│   └── ...
└── Videos/ (fallback for standalone videos only)
```

### 6. Cross-Course Compatibility
Explain how this system handles:
- Different course numbering (CS61A, CS229, EE106)
- Different institutions (Berkeley, Stanford, MIT)
- Different terminology (homework vs assignment vs problem set)
- Different file formats (all PDFs vs all HTML vs mixed)
- Video relationships (exam walkthroughs, lecture recordings, discussion videos)

## Output Format

Return a **complete markdown specification** focused on RULES for rearrangement.

**CRITICAL INSTRUCTIONS:**
- **DO include**: Detection rules, destination patterns, standardization rules, special handling
- **Use rule-based format**: "IF [condition] THEN [destination]"
- **Be concise**: Focus on actionable rules, not examples

Generate the specification now:"""

    client = OpenAI(api_key=api_key)

    print(f"Calling {model} to generate specification...")

    response = client.chat.completions.create(
        model=model,
        messages=[{"role": "user", "content": prompt}],
        temperature=0.3,
        max_tokens=16000
    )

    return response.choices[0].message.content.strip()


def generate_specification(
    scan_dir: str | Path,
    course_id: str,
    term: str,
    output: str | Path = "file_rearrangement_spec.md",
    sample_size: int = 100,
    model: str = "gpt-4.1",
    save_samples: bool = True
) -> Dict[str, Any]:
    """
    Generate a file rearrangement specification document.

    Args:
        scan_dir: Directory containing course files
        course_id: Course identifier
        term: Academic term
        output: Output path for markdown specification
        sample_size: Number of files to sample for analysis
        model: OpenAI model to use
        save_samples: Whether to save sampled files to JSON

    Returns:
        Dictionary with specification and sampled data
    """
    scan_dir = Path(scan_dir)
    output = Path(output)

    print(f"File Rearrangement Specification Generator")
    print(f"=" * 60)
    print(f"Course: {course_id} ({term})")
    print(f"Scanning: {scan_dir}")
    print(f"Output: {output}")
    print()

    # Step 1: Sample files
    print(f"Step 1: Sampling {sample_size} representative files...")
    sampled_data = sample_files_from_directory(scan_dir, sample_size)

    print(f"✓ Sampled {sampled_data['sampled_count']} files from {sampled_data['total_files']} total")
    print(f"  Found {sampled_data['statistics']['total_directories']} top-level directories")
    print()

    # Step 2: Generate specification with LLM
    print(f"Step 2: Generating specification with {model}...")
    specification_md = call_openai_for_spec(sampled_data, course_id, term, model)

    print(f"✓ Specification generated ({len(specification_md)} characters)")
    print()

    # Step 3: Save specification
    print(f"Step 3: Saving specification to {output}...")
    output.parent.mkdir(parents=True, exist_ok=True)

    with open(output, "w", encoding="utf-8") as f:
        f.write(specification_md)

    print(f"✓ Specification saved")
    print()

    # Step 4: Save samples if requested
    if save_samples:
        samples_path = output.parent / f"{output.stem}_samples.json"
        print(f"Step 4: Saving file samples to {samples_path}...")

        with open(samples_path, "w", encoding="utf-8") as f:
            json.dump(sampled_data, f, ensure_ascii=False, indent=2)

        print(f"✓ Samples saved")
        print()

    # Summary
    print("=" * 60)
    print("COMPLETE")
    print("=" * 60)
    print(f"Specification: {output}")
    if save_samples:
        print(f"Samples: {samples_path}")
    print()
    print("Next steps:")
    print("1. Review the generated specification document")
    print("2. Make any necessary adjustments")
    print("3. Use the specification to guide file rearrangement")

    return {
        "specification": specification_md,
        "sampled_data": sampled_data,
        "output_path": str(output)
    }


def main():
    """Main entry point for command-line usage."""
    config = {
        "scan_dir": "/home/bot/bot/yk/YK_final/courses/CS 61A",
        "course_id": "CS 61A",
        "term": "2025SM",
        "output": "CS61A_rearrangement_spec.md",
        "sample_size": 100,
        "model": "gpt-4.1",
        "save_samples": True
    }

    try:
        generate_specification(**config)
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        import sys
        sys.exit(1)


if __name__ == "__main__":
    main()
