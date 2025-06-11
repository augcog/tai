#!/usr/bin/env python3
"""
Database migration script: Legacy file_registry → Modern simplified schema
Safely migrates existing data to the new clean file model structure.
"""

import sqlite3
import sys
from pathlib import Path

# Add the app directory to the path so we can import our models
sys.path.append(str(Path(__file__).parent.parent))

from app.core.database import engine
from app.api.v1.models.files import FileRegistry, Base
from sqlalchemy import text
from sqlalchemy.orm import sessionmaker

def migrate_database():
    """
    Migrate the database from legacy over-engineered schema to modern clean schema
    """
    print("🔄 Starting database migration to modern file schema...")
    
    # Create a direct SQLite connection for schema operations
    db_path = "courses.db"
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    
    try:
        # Step 1: Check if old table exists
        cursor.execute("SELECT name FROM sqlite_master WHERE type='table' AND name='file_registry'")
        old_table_exists = cursor.fetchone() is not None
        
        if not old_table_exists:
            print("✅ No existing file_registry table found. Creating new schema...")
            # Create new tables using SQLAlchemy
            Base.metadata.create_all(bind=engine)
            print("✅ Modern file schema created successfully!")
            return True
        
        print("📋 Found existing file_registry table. Migrating data...")
        
        # Step 2: Backup existing data
        print("💾 Backing up existing data...")
        cursor.execute("""
            SELECT 
                id, file_name, relative_path, mime_type, size_bytes,
                course_code, category, title, is_active, created_at, updated_at
            FROM file_registry 
            WHERE is_active = 1
        """)
        existing_data = cursor.fetchall()
        print(f"📊 Found {len(existing_data)} active files to migrate")
        
        # Step 3: Rename old table
        print("🔄 Renaming old table...")
        cursor.execute("ALTER TABLE file_registry RENAME TO file_registry_backup")
        
        # Step 4: Create new table with modern schema
        print("🏗️  Creating new modern schema...")
        conn.close()  # Close SQLite connection
        
        # Use SQLAlchemy to create the new schema
        Base.metadata.create_all(bind=engine)
        
        # Step 5: Migrate data to new schema
        print("📦 Migrating data to new schema...")
        Session = sessionmaker(bind=engine)
        session = Session()
        
        migrated_count = 0
        for row in existing_data:
            try:
                # Map old columns to new schema
                file_record = FileRegistry(
                    id=row[0],  # id (UUID)
                    file_name=row[1],  # file_name
                    relative_path=row[2],  # relative_path
                    mime_type=row[3],  # mime_type
                    size_bytes=row[4],  # size_bytes
                    course_code=row[5],  # course_code
                    category=row[6] if row[6] in ['document', 'video', 'audio', 'other'] else 'other',  # category (clean)
                    title=row[7],  # title
                    is_active=bool(row[8]),  # is_active
                    created_at=row[9],  # created_at
                    modified_at=row[10]  # updated_at → modified_at
                )
                session.add(file_record)
                migrated_count += 1
                
                if migrated_count % 10 == 0:
                    session.commit()  # Commit in batches
                    
            except Exception as e:
                print(f"⚠️  Warning: Could not migrate file {row[1]}: {e}")
                continue
        
        session.commit()
        session.close()
        
        print(f"✅ Successfully migrated {migrated_count} files to modern schema!")
        
        # Step 6: Verify migration
        print("🔍 Verifying migration...")
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()
        cursor.execute("SELECT COUNT(*) FROM file_registry")
        new_count = cursor.fetchone()[0]
        print(f"📊 New table contains {new_count} files")
        
        if new_count == migrated_count:
            print("✅ Migration verification successful!")
            
            # Optional: Remove backup table (commented out for safety)
            # print("🗑️  Removing backup table...")
            # cursor.execute("DROP TABLE file_registry_backup")
            print("💾 Backup table 'file_registry_backup' preserved for safety")
        else:
            print("❌ Migration verification failed! Check data integrity.")
            return False
            
        conn.close()
        return True
        
    except Exception as e:
        print(f"❌ Migration failed: {e}")
        conn.rollback()
        return False
    finally:
        conn.close()


def check_migration_needed():
    """Check if migration is needed"""
    try:
        conn = sqlite3.connect("courses.db")
        cursor = conn.cursor()
        
        # Check if the new schema exists
        cursor.execute("PRAGMA table_info(file_registry)")
        columns = [col[1] for col in cursor.fetchall()]
        
        # Check if we have the new schema (modified_at) or old schema (updated_at)
        has_modified_at = 'modified_at' in columns
        has_updated_at = 'updated_at' in columns
        has_over_engineering = any(col in columns for col in ['assignment_number', 'week_number', 'subcategory'])
        
        conn.close()
        
        if has_modified_at and not has_over_engineering:
            print("✅ Database already has modern schema. No migration needed.")
            return False
        elif has_updated_at or has_over_engineering:
            print("🔄 Legacy schema detected. Migration needed.")
            return True
        else:
            print("🆕 No file_registry table found. Will create new schema.")
            return True
            
    except Exception as e:
        print(f"⚠️  Could not check database schema: {e}")
        return True


if __name__ == "__main__":
    print("🚀 File Registry Migration Tool")
    print("=" * 50)
    
    if check_migration_needed():
        print("\n🔄 Starting migration process...")
        success = migrate_database()
        
        if success:
            print("\n🎉 Migration completed successfully!")
            print("✅ Your file registry now uses the modern, clean schema.")
            print("💾 Original data backed up as 'file_registry_backup' table.")
        else:
            print("\n❌ Migration failed!")
            print("🔧 Please check the error messages above and try again.")
            sys.exit(1)
    else:
        print("\n✅ No migration needed. Database is already up to date.")
    
    print("\n🚀 Ready to use the modern Files API!")
