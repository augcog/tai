#!/usr/bin/env python3
"""
Database and File Initialization Script

This script provides a command-line interface for database initialization and file importing.
It can be run manually or automatically called by the server on startup.

Usage:
    python scripts/initialize_db_and_files.py              # Full initialization
    python scripts/initialize_db_and_files.py --check      # Check database status only
    python scripts/initialize_db_and_files.py --force      # Force reimport of all files
    python scripts/initialize_db_and_files.py --data-dir custom_data  # Use custom data directory
"""

from app.core.db_initializer import get_initializer
import sys
import argparse
from pathlib import Path

# Add the parent directory to sys.path to import app modules
sys.path.append(str(Path(__file__).parent.parent))


def main():
    """Main CLI function"""
    parser = argparse.ArgumentParser(
        description="Initialize database and import files from data directory",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s                          # Full initialization
  %(prog)s --check                  # Check database status
  %(prog)s --force                  # Force reimport all files
  %(prog)s --data-dir /path/to/data # Use custom data directory
        """,
    )

    parser.add_argument(
        "--data-dir",
        type=str,
        default="data",
        help="Data directory path (default: data)",
    )

    parser.add_argument(
        "--check",
        action="store_true",
        help="Check database status without making changes",
    )

    parser.add_argument(
        "--force",
        action="store_true",
        help="Force reimport of all files (clears existing file registry first)",
    )

    parser.add_argument("--verbose", action="store_true", help="Enable verbose logging")

    args = parser.parse_args()

    # Configure logging level
    import logging

    if args.verbose:
        logging.basicConfig(level=logging.DEBUG)
    else:
        logging.basicConfig(level=logging.INFO)

    print("🚀 Database and File Initialization Tool")
    print("=" * 50)

    # Initialize the database initializer
    initializer = get_initializer(args.data_dir)

    if args.check:
        # Check database status
        print("🔍 Checking database status...")
        status = initializer.get_database_status()

        if "error" in status:
            print(f"❌ Error checking database: {status['error']}")
            return 1

        print("\n📊 Database Status Report:")
        print(
            f"  Database exists: {'✅ Yes' if status['database_exists'] else '❌ No'}"
        )
        print(f"  Database size: {status['database_size_bytes']:,} bytes")
        print(f"  Files in registry: {status['file_count']:,}")
        print(f"  Registered courses: {status['course_count']:,}")
        print(
            f"  Data directory exists: {'✅ Yes' if status['data_directory_exists'] else '❌ No'}"
        )

        if status["data_directory_exists"]:
            # Count files in data directory
            data_path = Path(args.data_dir)
            file_count = len(
                [
                    f
                    for f in data_path.rglob("*")
                    if f.is_file() and not f.name.startswith(".")
                ]
            )
            print(f"  Files in data directory: {file_count:,}")

        return 0

    elif args.force:
        # Force reimport - clear existing file registry first
        print("⚠️  Force mode: This will clear the existing file registry!")
        confirm = input("Are you sure you want to continue? (y/N): ")

        if confirm.lower() != "y":
            print("❌ Operation cancelled.")
            return 0

        print("🗑️  Clearing existing file registry...")
        session = initializer.SessionLocal()
        try:
            from app.core.models.files import FileRegistry

            session.query(FileRegistry).delete()
            session.commit()
            print("✅ File registry cleared.")
        except Exception as e:
            session.rollback()
            print(f"❌ Failed to clear file registry: {e}")
            return 1
        finally:
            session.close()

        # Now run full initialization
        print("🔄 Running full initialization...")
        success = initializer.initialize_database()

        if success:
            print("\n🎉 Force initialization completed successfully!")
            # Show final status
            status = initializer.get_database_status()
            print(f"📊 Files imported: {status['file_count']:,}")
            print(f"📚 Courses registered: {status['course_count']:,}")
        else:
            print("\n❌ Force initialization failed!")
            return 1

    else:
        # Normal initialization
        print("🔄 Running database initialization...")
        success = initializer.initialize_database()

        if success:
            print("\n🎉 Database initialization completed successfully!")

            # Show final status
            status = initializer.get_database_status()
            print("\n📊 Final Status:")
            print(f"  Files in registry: {status['file_count']:,}")
            print(f"  Registered courses: {status['course_count']:,}")
            print(f"  Database size: {status['database_size_bytes']:,} bytes")

        else:
            print("\n❌ Database initialization failed!")
            print("💡 Try running with --verbose for more details")
            return 1

    print("\n✅ Ready to start the server!")
    print("💡 Next steps:")
    print("   1. Start the server: python main.py")
    print("   2. Visit http://localhost:8000/docs for API documentation")
    return 0


if __name__ == "__main__":
    exit(main())
