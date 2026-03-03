#!/usr/bin/env python3  
from __future__ import annotations

import argparse
import json
import os
import re
import sys
from pathlib import Path
from typing import List, Dict, Any

# Load environment variables from .env file
try:
    from dotenv import load_dotenv
    # Try multiple possible locations for .env file
    possible_env_paths = [
        Path(__file__).parent.parent / '.env',  # file_conversion_router/.env
        Path(__file__).parent.parent.parent / '.env',  # rag/.env
        Path.cwd() / '.env',  # current working directory
        Path.cwd().parent / '.env',  # parent of current working directory
    ]
    
    env_loaded = False
    for env_path in possible_env_paths:
        if env_path.exists():
            load_dotenv(env_path)
            print(f"Loaded environment variables from: {env_path}")
            env_loaded = True
            break
    
    if not env_loaded:
        print("Warning: No .env file found in expected locations", file=sys.stderr)
        
except ImportError:
    # If python-dotenv is not available, continue without it
    print("Warning: python-dotenv not available, environment variables not loaded", file=sys.stderr)

ALLOWED_TYPES = {
    "lecture_slide","reading","assignment","lab","project","notes","exam","code","media"
}

PROMPT_TEMPLATE = r"""
You are an expert educational content organizer. Your job is to analyze the course syllabus and generate a TOPIC-BASED/MODULE-BASED organization structure where each module represents a lecture topic with all associated materials.

### Task
Students navigate by MODULE/TOPIC aligned with lecture sequence. Each module contains ALL materials for that lecture topic:
- Module 01: Introduction to Programming
  - Lecture slides (lecture_01.pdf)
  - Readings (Chapter 1)
  - Homework (hw01.pdf)
  - Lab (lab01.pdf)

Based on the syllabus, identify the lecture sequence and group all related materials into logical modules.

### JSON Schema
Generate a JSON object with this exact structure:

{{
  "course_id": "{course_id}",
  "term": "{term}",
  "structure_type": "topic_based",
  "modules": [
    {{
      "module_id": "module_01",
      "lecture_number": "01",
      "title": "Introduction to Programming",
      "topics": ["Variables", "Functions", "Control Flow"],
      "description": "Introduction to basic programming concepts including variables, functions, and control structures",
      "week": 1,
      "aliases": ["lecture 01", "lec01", "week 1", "module 1", "intro"],
      "expected_materials": {{
        "lecture_slides": true,
        "readings": ["Chapter 1: Introduction"],
        "assignments": ["HW01"],
        "labs": ["Lab01"],
        "projects": [],
        "discussions": [],
        "other": []
      }}
    }},
    {{
      "module_id": "module_02",
      "lecture_number": "02",
      "title": "Data Structures",
      "topics": ["Lists", "Dictionaries", "Tuples"],
      "description": "Working with fundamental data structures in Python",
      "week": 2,
      "aliases": ["lecture 02", "lec02", "week 2", "module 2", "data structures"],
      "expected_materials": {{
        "lecture_slides": true,
        "readings": ["Chapter 2: Data Structures"],
        "assignments": ["HW02"],
        "labs": ["Lab02"],
        "projects": [],
        "discussions": ["Discussion 02"],
        "other": []
      }}
    }}
  ]
}}

### Organization Rules - TOPIC-BASED/MODULE-BASED STRUCTURE

Each module should align with a lecture and contain all associated course materials for that topic.

**For each module:**
- **module_id**: Unique identifier in format "module_XX" (e.g., "module_01", "module_02")
- **lecture_number**: The lecture number this module corresponds to (e.g., "01", "02", "03")
- **title**: Descriptive title of the lecture topic (e.g., "Introduction to Programming")
- **topics**: List of 2-5 specific topics covered in this module
- **description**: 1-2 sentence description of what this module covers
- **week**: Week number in the course (integer, e.g., 1, 2, 3)
- **aliases**: Alternative names/identifiers for this module
  - Include: lecture number variations ("lecture 01", "lec01", "L01")
  - Week identifiers ("week 1", "week 01")
  - Module identifiers ("module 1", "mod 1")
  - Topic keywords from the title (lowercase, e.g., "intro", "recursion")
- **expected_materials**: Dictionary specifying what materials belong to this module
  - **lecture_slides**: boolean (true if lecture exists)
  - **readings**: List of reading titles/chapters (empty list [] if none)
  - **assignments**: List of assignment names (e.g., ["HW01", "HW02"])
  - **labs**: List of lab names (e.g., ["Lab01"])
  - **projects**: List of project names (e.g., ["Project 1: Hog"])
  - **discussions**: List of discussion names (e.g., ["Discussion 01"])
  - **other**: Any other materials (e.g., ["Study Guide", "Practice Exam"])

### Extraction Guidelines

1. **Identify Lecture Sequence**: Look for lecture numbers, dates, or weekly schedule
2. **Extract Topics**: Identify the main topics/concepts for each lecture
3. **Map Materials to Lectures**: Associate each assignment, lab, reading with the appropriate lecture/week
4. **Group by Module**: Create one module per lecture, containing all related materials
5. **Handle Special Cases**:
   - If multiple lectures cover one topic, create separate modules for each lecture
   - If one lecture covers multiple distinct topics, list all topics
   - Projects may span multiple modules - list them in the primary module
   - Exams and reviews get their own modules (e.g., "module_midterm", "module_final")

### Mapping Strategy

**From Syllabus to Modules:**
- Week 1, Lecture 1: "Introduction" → Module 01
  - Materials: Lecture 1 slides, HW01, Lab01, Reading Chapter 1
- Week 2, Lecture 2: "Functions" → Module 02
  - Materials: Lecture 2 slides, HW02, Lab02, Reading Chapter 2
- Week 5: "Midterm" → module_midterm
  - Materials: Midterm exam, study guide, practice problems

**Reading Assignment Extraction:**
- "Read Chapter 1" → ["Chapter 1"]
- "SICP 1.1-1.3" → ["SICP Sections 1.1-1.3"]
- "Paper: MapReduce" → ["Paper: MapReduce"]

**Assignment Pattern Recognition:**
- "HW 1 due Week 2" → Assign to Module 02
- "Lab 3 covers Lecture 3 material" → Assign to Module 03
- "Project 1 (Weeks 3-5)" → Assign to Module 03 (start week)

### Output Requirements
- Output **only valid JSON** with no extra commentary
- Keys must match schema exactly
- Include 8-15 modules typically (one per lecture, plus exams/reviews)
- Modules should be in chronological order by lecture number
- Use consistent numbering (module_01, module_02, not module_1, module_2)
- Aliases should be comprehensive (<40 characters each)
- For expected_materials, use empty lists [] when materials don't exist, not null

### Special Module Types

**Regular Lecture Module**: Standard lecture with materials
**Midterm Module**:
- module_id: "module_midterm"
- lecture_number: "midterm"
- title: "Midterm Exam"
- expected_materials: {{"lecture_slides": false, "readings": [], "assignments": [], "labs": [], "projects": [], "discussions": [], "other": ["Midterm Exam", "Study Guide"]}}

**Final Module**:
- module_id: "module_final"
- lecture_number: "final"
- title: "Final Exam"

**Review Session**:
- Treat as regular module with descriptive title

### Course Metadata
course_id = "{course_id}"
term = "{term}"

### Raw Syllabus Text
<<<RAW_SYLLABUS_START
{raw_text}
RAW_SYLLABUS_END
"""

def preprocess_markdown(md_text: str) -> str:
    """
    Preprocess markdown to extract clean text suitable for LLM processing.
    Removes excessive formatting while preserving structure.
    """
    # Remove HTML comments
    text = re.sub(r'<!--.*?-->', '', md_text, flags=re.DOTALL)

    # Convert headers to plain text with proper spacing
    text = re.sub(r'^#{1,6}\s+', '\n', text, flags=re.MULTILINE)

    # Remove image references but keep alt text
    text = re.sub(r'!\[([^\]]*)\]\([^\)]+\)', r'\1', text)

    # Convert links to plain text (keep link text, discard URL)
    text = re.sub(r'\[([^\]]+)\]\([^\)]+\)', r'\1', text)

    # Remove inline code backticks
    text = re.sub(r'`([^`]+)`', r'\1', text)

    # Remove code blocks
    text = re.sub(r'```.*?```', '', text, flags=re.DOTALL)

    # Remove horizontal rules
    text = re.sub(r'^[-*_]{3,}$', '', text, flags=re.MULTILINE)

    # Normalize whitespace
    text = re.sub(r'\n{3,}', '\n\n', text)
    text = re.sub(r'[ \t]+', ' ', text)

    return text.strip()

def extract_json(text: str) -> Dict[str, Any]:
    """
    Extract JSON from LLM output with multiple fallback strategies.
    Handles markdown code blocks, plain JSON, and embedded JSON.
    """
    text = text.strip()

    # Strategy 1: Try parsing as-is
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        pass

    # Strategy 2: Extract from markdown JSON code block
    patterns = [
        r"```json\s*(\{.*?\})\s*```",  # ```json {...} ```
        r"```\s*(\{.*?\})\s*```",       # ``` {...} ```
    ]
    for pattern in patterns:
        m = re.search(pattern, text, flags=re.DOTALL)
        if m:
            try:
                return json.loads(m.group(1))
            except json.JSONDecodeError:
                continue

    # Strategy 3: Find first complete JSON object
    start = text.find('{')
    if start != -1:
        # Count braces to find matching closing brace
        depth = 0
        for i in range(start, len(text)):
            if text[i] == '{':
                depth += 1
            elif text[i] == '}':
                depth -= 1
                if depth == 0:
                    try:
                        return json.loads(text[start:i+1])
                    except json.JSONDecodeError:
                        pass
                    break

    # Strategy 4: Last resort - find outermost braces
    end = text.rfind('}')
    if start != -1 and end != -1 and end > start:
        try:
            return json.loads(text[start:end+1])
        except json.JSONDecodeError:
            pass

    raise ValueError(f"No valid JSON found in model output. Output preview: {text[:200]}...")

def normalize_json(obj: Dict[str, Any], course_id: str, term: str) -> Dict[str, Any]:
    obj["course_id"] = course_id
    obj["term"] = term
    obj["structure_type"] = "topic_based"

    modules = obj.get("modules") or []
    if not isinstance(modules, list):
        modules = []
    if len(modules) < 5:
        print("[warn] fewer than 5 modules; consider improving the prompt", file=sys.stderr)
    if len(modules) > 20:
        print(f"[warn] more than 20 modules ({len(modules)}), truncating to 20", file=sys.stderr)
        modules = modules[:20]

    normalized = []
    for i, m in enumerate(modules, 1):
        n = {}

        # Module ID (e.g., "module_01", "module_midterm")
        module_id = m.get("module_id", "").strip().lower()
        if not module_id:
            module_id = f"module_{i:02d}"
        n["module_id"] = module_id

        # Lecture number
        lecture_num = m.get("lecture_number", "").strip()
        if not lecture_num:
            lecture_num = f"{i:02d}"
        n["lecture_number"] = lecture_num

        # Title and description
        n["title"] = (m.get("title") or "Untitled Module").strip()
        n["description"] = (m.get("description") or "").strip()

        # Topics (list of strings)
        topics = m.get("topics", [])
        if not isinstance(topics, list):
            topics = []
        n["topics"] = [t.strip() for t in topics if isinstance(t, str) and t.strip()][:10]

        # Week number
        week = m.get("week")
        if isinstance(week, int):
            n["week"] = week
        elif isinstance(week, str) and week.isdigit():
            n["week"] = int(week)
        else:
            n["week"] = i

        # Aliases
        aliases = [a.strip() for a in m.get("aliases", []) if isinstance(a, str) and len(a.strip()) < 40]
        seen = set()
        aliases = [a for a in aliases if not (a.lower() in seen or seen.add(a.lower()))]
        n["aliases"] = aliases[:20]

        # Expected materials
        expected_materials = m.get("expected_materials", {})
        if not isinstance(expected_materials, dict):
            expected_materials = {}

        normalized_materials = {
            "lecture_slides": bool(expected_materials.get("lecture_slides", True)),
            "readings": [r.strip() for r in expected_materials.get("readings", []) if isinstance(r, str)][:10],
            "assignments": [a.strip() for a in expected_materials.get("assignments", []) if isinstance(a, str)][:10],
            "labs": [l.strip() for l in expected_materials.get("labs", []) if isinstance(l, str)][:10],
            "projects": [p.strip() for p in expected_materials.get("projects", []) if isinstance(p, str)][:10],
            "discussions": [d.strip() for d in expected_materials.get("discussions", []) if isinstance(d, str)][:10],
            "other": [o.strip() for o in expected_materials.get("other", []) if isinstance(o, str)][:10]
        }
        n["expected_materials"] = normalized_materials

        normalized.append(n)

    obj["modules"] = normalized
    return obj

def call_openai(prompt: str, model: str, api_key: str | None = None) -> str:
    """
    Call OpenAI API with the given prompt.
    Uses chat completions API (correct endpoint).
    """
    from openai import OpenAI
    api_key = api_key or os.getenv("OPENAI_API_KEY")
    if not api_key:
        raise RuntimeError("OPENAI_API_KEY not set")

    client = OpenAI(api_key=api_key)

    try:
        resp = client.chat.completions.create(
            model=model,
            messages=[{"role": "user", "content": prompt}],
            temperature=0.2,
            max_tokens=4000,
            response_format={"type": "json_object"}  # Request JSON output
        )
        return resp.choices[0].message.content.strip()
    except Exception as e:
        raise RuntimeError(f"OpenAI API call failed: {e}")

def validate_input_file(file_path: str) -> str:
    """
    Validate and read input file.
    Supports .md, .txt, and plain text files.
    """
    path = Path(file_path)
    if not path.exists():
        raise FileNotFoundError(f"Input file not found: {file_path}")

    if not path.is_file():
        raise ValueError(f"Input path is not a file: {file_path}")

    # Check file extension
    if path.suffix.lower() not in ['.md', '.txt', '.markdown', '']:
        print(f"[warn] Unexpected file extension '{path.suffix}'. Expected .md or .txt", file=sys.stderr)

    return path.read_text(encoding="utf-8")

def main():
    ap = argparse.ArgumentParser(
        description="Generate topic-based syllabus.json from course syllabus file. Organizes materials by lecture topics/modules, where each module contains all associated materials.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Example usage:
  python syllabus_builder.py --in syllabus.md --out syllabus.json --course-id EE106B --term 2025FA
  python syllabus_builder.py --in syllabus.txt --out output.json --course-id CS61A --term 2024SP --model gpt-4o

Output structure:
  {
    "course_id": "CS61A",
    "term": "2025FA",
    "structure_type": "topic_based",
    "modules": [
      {
        "module_id": "module_01",
        "lecture_number": "01",
        "title": "Introduction to Programming",
        "topics": ["Variables", "Functions", "Control Flow"],
        "description": "Introduction to basic programming concepts",
        "week": 1,
        "aliases": ["lecture 01", "lec01", "week 1", "module 1", "intro"],
        "expected_materials": {
          "lecture_slides": true,
          "readings": ["Chapter 1: Introduction"],
          "assignments": ["HW01"],
          "labs": ["Lab01"],
          "projects": [],
          "discussions": [],
          "other": []
        }
      },
      ...
    ]
  }
        """
    )
    ap.add_argument("--in", dest="inp", required=True, help="Input markdown/text file path")
    ap.add_argument("--out", dest="out", required=True, help="Output JSON file path")
    ap.add_argument("--course-id", dest="course_id", required=True, help="Course ID (e.g., EE106B, CS61A)")
    ap.add_argument("--term", required=True, help="Term (e.g., 2025FA, 2024SP)")
    ap.add_argument("--model", default="gpt-4o", help="OpenAI model to use (default: gpt-4o)")
    ap.add_argument("--skip-preprocess", action="store_true", help="Skip markdown preprocessing")
    args = ap.parse_args()

    try:
        # Read and validate input
        print(f"Reading input file: {args.inp}")
        raw = validate_input_file(args.inp)

        # Preprocess markdown if needed
        if not args.skip_preprocess and Path(args.inp).suffix.lower() in ['.md', '.markdown']:
            print("Preprocessing markdown to clean text...")
            raw = preprocess_markdown(raw)

        # Generate prompt and call OpenAI
        print(f"Calling OpenAI API (model: {args.model})...")
        prompt = PROMPT_TEMPLATE.format(
            course_id=args.course_id,
            term=args.term,
            raw_text=raw
        )
        text = call_openai(prompt, args.model)

        # Extract and normalize JSON
        print("Extracting JSON from response...")
        obj = extract_json(text)
        obj = normalize_json(obj, args.course_id, args.term)

        # Write output
        with open(args.out, "w", encoding="utf-8") as f:
            json.dump(obj, f, ensure_ascii=False, indent=2)

        print(f"✓ Successfully wrote {args.out} with {len(obj.get('modules', []))} modules.")
        print(f"  Structure type: topic_based")
        print(f"  Modules: {', '.join([m.get('module_id', '') for m in obj.get('modules', [])])}")
        print(f"  Lectures covered: {', '.join([m.get('lecture_number', '') for m in obj.get('modules', [])])}")

    except FileNotFoundError as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)
    except RuntimeError as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)
    except ValueError as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"Unexpected error: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()
