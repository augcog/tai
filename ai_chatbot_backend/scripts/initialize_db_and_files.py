#!/usr/bin/env python3
"""
Database Initialization Script

This script provides a command-line interface for database initialization.
It can be run manually or automatically called by the server on startup.

Usage:
    python scripts/initialize_db_and_files.py              # Full initialization
    python scripts/initialize_db_and_files.py --check      # Check database status only
    python scripts/initialize_db_and_files.py --force      # Force clear and reinitialize from MongoDB
"""

from app.core.dbs.db_initializer import get_initializer
import sys
import argparse
from pathlib import Path

# Add the parent directory to sys.path to import app modules
sys.path.append(str(Path(__file__).parent.parent))


def main():
    """Main CLI function"""
    parser = argparse.ArgumentParser(
        description="Initialize database and load data from MongoDB",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s                          # Full initialization
  %(prog)s --check                  # Check database status
  %(prog)s --force                  # Force clear and reinitialize
        """,
    )


    parser.add_argument(
        "--check",
        action="store_true",
        help="Check database status without making changes",
    )

    parser.add_argument(
        "--force",
        action="store_true",
        help="Force clear and reinitialize databases from MongoDB",
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
    initializer = get_initializer()

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
        print(f"  Registered courses: {status['course_count']:,}")

        return 0

    elif args.force:
        # Force clear and reinitialize from MongoDB
        print("⚠️  Force mode: This will clear and reinitialize databases from MongoDB!")
        confirm = input("Are you sure you want to continue? (y/N): ")

        if confirm.lower() != "y":
            print("❌ Operation cancelled.")
            return 0

        print("🗑️  Clearing existing databases...")
        try:
            # Remove database files
            from app.core.dbs.course_db import COURSE_DATABASE_URL
            from app.core.dbs.metadata_db import METADATA_DATABASE_URL
            
            courses_db_path = Path(COURSE_DATABASE_URL.replace("sqlite:///", ""))
            metadata_db_path = Path(METADATA_DATABASE_URL.replace("sqlite:///", ""))
            
            if courses_db_path.exists():
                courses_db_path.unlink()
                print("✅ Courses database cleared.")
            
            if metadata_db_path.exists():
                metadata_db_path.unlink()
                print("✅ Metadata database cleared.")
                
        except Exception as e:
            print(f"❌ Failed to clear databases: {e}")
            return 1

        # Now run full initialization
        print("🔄 Running full initialization from MongoDB...")
        success = initializer.initialize_database()

        if success:
            print("\n🎉 Force initialization completed successfully!")
            # Show final status
            status = initializer.get_database_status()
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
