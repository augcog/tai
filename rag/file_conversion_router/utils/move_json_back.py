#!/usr/bin/env python3
import argparse
import shutil
from pathlib import Path

def derive_dest(json_path: Path, src_root: Path, dst_root: Path, strip_video_ext: bool):
    """
    Map:
      courses_out/<COURSE>_output/.../youtube/.../<FILENAME>/<FILENAME>.<ext>.json
    -> courses/<COURSE>/.../youtube/.../<FILENAME>.<ext>.json  (same name by default)
       or courses/<COURSE>/.../youtube/.../<FILENAME>.json     (if strip_video_ext)
    """
    rel = json_path.relative_to(src_root)  # e.g. CS 61A_output/youtube/.../1-Tail Recursion/1-Tail Recursion.webm.json
    parts = list(rel.parts)
    if len(parts) < 4:
        return None, f"Path too short: {json_path}"

    course_out = parts[0]
    if not course_out.endswith("_output"):
        return None, f"Course segment missing _output: {json_path}"
    course = course_out[:-7]  # drop "_output"

    # Expect last two parts: <FILENAME_DIR>/<FILENAME>.<ext>.json
    filename_dir = parts[-2]
    json_filename = parts[-1]
    if not json_filename.endswith(".json"):
        return None, f"Not a JSON file: {json_path}"

    # Validate the structure: the filename starts with the same base as the parent dir
    name_without_json = json_filename[:-5]  # strip .json
    # If output keeps original video extension inside the name, we can strip it optionally
    base_without_ext = Path(name_without_json).stem  # strips one extension (e.g., .webm)
    if filename_dir != base_without_ext:
        # Not fatal, but likely means structure differs from assumption
        return None, f"Filename base != parent dir ({filename_dir} vs {base_without_ext}) for {json_path}"

    # Build destination directory:
    # Replace root and course segment, drop the extra filename_dir level
    middle = parts[1:-2]  # e.g., ["youtube", "61A Fall 2023 Lecture 32"]
    dest_dir = dst_root / course
    for seg in middle:
        dest_dir = dest_dir / seg

    # Decide destination filename
    if strip_video_ext:
        dest_name = f"{base_without_ext}.json"
    else:
        dest_name = json_filename  # keep e.g. 1-Tail Recursion.webm.json

    dest_path = dest_dir / dest_name
    return dest_path, None

def main():
    parser = argparse.ArgumentParser(description="Move generated JSON files from courses_out back into original courses tree.")
    parser.add_argument("--src-root", required=True, help="Root of output tree (e.g., /home/.../courses_out)")
    parser.add_argument("--dst-root", required=True, help="Root of original tree (e.g., /home/.../courses)")
    parser.add_argument("--pattern", default="*.json", help="Glob for JSON files (default: *.json)")
    parser.add_argument("--strip-video-ext", action="store_true",
                        help="Rename <name>.<videoext>.json -> <name>.json when moving.")
    parser.add_argument("--overwrite", action="store_true", help="Overwrite existing destination files.")
    parser.add_argument("--dry-run", action="store_true", help="Show what would happen without moving files.")
    parser.add_argument("--prune-empty-dirs", action="store_true", help="Remove empty dirs left in src after moves.")
    args = parser.parse_args()

    src_root = Path(args.src_root).resolve()
    dst_root = Path(args.dst_root).resolve()

    moved, skipped, errors = 0, 0, 0
    to_prune = set()

    for json_path in src_root.rglob(args.pattern):
        if not json_path.is_file():
            continue
        dest_path, err = derive_dest(json_path, src_root, dst_root, args.strip_video_ext)
        if err:
            print(f"[SKIP] {err}")
            skipped += 1
            continue

        dest_path.parent.mkdir(parents=True, exist_ok=True)

        if dest_path.exists() and not args.overwrite:
            print(f"[SKIP] Exists (use --overwrite): {dest_path}")
            skipped += 1
            continue

        print(f"{'MOVE' if not args.dry_run else 'DRY'}: {json_path} -> {dest_path}")
        if not args.dry_run:
            try:
                # Use move to allow cross-device moves
                shutil.move(str(json_path), str(dest_path))
                moved += 1
                to_prune.add(json_path.parent)  # candidate for pruning
            except Exception as e:
                print(f"[ERROR] Moving {json_path} -> {dest_path}: {e}")
                errors += 1

    if args.prune_empty_dirs and not args.dry_run:
        # Attempt to prune empty source directories (bottom-up)
        for d in sorted(to_prune, key=lambda p: len(p.as_posix()), reverse=True):
            try:
                # Only remove if empty (raises OSError if not empty)
                d.rmdir()
                # Also try to remove parent if it became empty (but stay inside src_root)
                parent = d.parent
                while src_root in parent.parents or parent == src_root:
                    try:
                        parent.rmdir()
                        parent = parent.parent
                    except OSError:
                        break
            except OSError:
                pass

    print(f"\nDone. Moved: {moved}, Skipped: {skipped}, Errors: {errors}")




if __name__ == "__main__":
    main()
