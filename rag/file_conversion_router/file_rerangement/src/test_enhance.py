#!/usr/bin/env python3
"""
Test script for enhancing existing syllabus with new multi-stage pipeline
"""

from folder_structure_generator import enhance_existing_syllabus
from pathlib import Path

# Paths
SYLLABUS_PATH = Path(__file__).parent / "syllabus.json"
OUTPUT_PATH = Path(__file__).parent / "syllabus_enhanced.json"

def main():
    print("=" * 80)
    print("Testing Multi-Stage Pipeline Enhancement")
    print("=" * 80)

    # Test enhancement
    try:
        syllabus = enhance_existing_syllabus(
            syllabus_path=SYLLABUS_PATH,
            output_path=OUTPUT_PATH,
            model="gpt-4o"
        )

        print("\n" + "=" * 80)
        print("ENHANCEMENT RESULTS")
        print("=" * 80)

        for unit in syllabus.get("units", []):
            print(f"\nUnit: {unit['title']}")
            print(f"  Unit ID: {unit['unit_id']}")

            structure_design = unit.get("structure_design", {})
            print(f"  Organization Type: {structure_design.get('organization_type', 'N/A')}")

            subfolders = structure_design.get("structure", {})
            if subfolders:
                print(f"  Subfolders ({len(subfolders)}):")
                for subfolder_name, info in list(subfolders.items())[:3]:
                    print(f"    - {subfolder_name}: {info.get('description', 'N/A')}")
                if len(subfolders) > 3:
                    print(f"    ... and {len(subfolders) - 3} more")

            path_mappings = unit.get("path_mappings", [])
            print(f"  Path Mappings: {len(path_mappings)} files")

            if path_mappings:
                print(f"  Sample mappings:")
                for mapping in path_mappings[:3]:
                    print(f"    {mapping['source_path']}")
                    print(f"    → {mapping['dest_path']}")
                if len(path_mappings) > 3:
                    print(f"    ... and {len(path_mappings) - 3} more")

        print("\n" + "=" * 80)
        print(f"✓ SUCCESS: Enhanced syllabus saved to {OUTPUT_PATH}")
        print("=" * 80)

    except Exception as e:
        print(f"\n✗ ERROR: {e}")
        import traceback
        traceback.print_exc()
        return 1

    return 0

if __name__ == "__main__":
    exit(main())
