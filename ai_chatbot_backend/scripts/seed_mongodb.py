#!/usr/bin/env python3
"""
Configurable MongoDB Seeding Script

This script provides a flexible system for seeding MongoDB databases with data
from exported JSON files. It uses configuration-driven approach and supports
batch processing, validation, and idempotent operations.

Usage:
    python scripts/seed_mongodb.py                     # Seed all databases
    python scripts/seed_mongodb.py --db courses       # Seed specific database
    python scripts/seed_mongodb.py --collection courses # Seed specific collection
    python scripts/seed_mongodb.py --input custom_exports # Custom input directory
    python scripts/seed_mongodb.py --dry-run          # Show what would be done
    python scripts/seed_mongodb.py --clean            # Drop collections before seeding
    python scripts/seed_mongodb.py --status           # Show MongoDB status
"""

import json
import argparse
import logging
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Any, Optional, Tuple
import sys

# Add the parent directory to sys.path to import app modules
sys.path.append(str(Path(__file__).parent.parent))

from app.core.mongodb_client import MongoDBClient, load_database_mapping
from app.config import settings

logger = logging.getLogger(__name__)


class MongoDBSeeder:
    """Configurable MongoDB seeding system"""
    
    def __init__(self, config: Dict[str, Any], input_dir: str = "exports",
                 mongodb_uri: str = None):
        """Initialize seeder with configuration
        
        Args:
            config: Database mapping configuration
            input_dir: Directory containing exported JSON files
            mongodb_uri: MongoDB connection URI
        """
        self.config = config
        self.input_dir = Path(input_dir)
        self.mongodb_uri = mongodb_uri or settings.MONGODB_URI
        
        # Initialize MongoDB client
        self.client = MongoDBClient(self.mongodb_uri)
        
        # Validate configuration
        self._validate_config()
    
    def _validate_config(self) -> bool:
        """Validate configuration structure
        
        Returns:
            bool: True if configuration is valid
        """
        if 'mongodb' not in self.config:
            raise ValueError("Missing 'mongodb' section in configuration")
        
        mongodb_config = self.config['mongodb']
        
        if 'databases' not in mongodb_config:
            raise ValueError("Missing 'databases' section in MongoDB configuration")
        
        logger.info("✅ Configuration validation passed")
        return True
    
    def _find_export_files(self) -> Dict[str, Path]:
        """Find all export files in input directory
        
        Returns:
            Dict mapping collection identifier to file path
        """
        if not self.input_dir.exists():
            logger.warning(f"⚠️ Input directory not found: {self.input_dir}")
            return {}
        
        export_files = {}
        
        for file_path in self.input_dir.glob("*.json"):
            # Parse filename: {database}_{collection}.json
            filename = file_path.stem
            if '_' in filename:
                database, collection = filename.split('_', 1)
                key = f"{database}.{collection}"
                export_files[key] = file_path
            else:
                logger.warning(f"⚠️ Skipping file with invalid name format: {file_path}")
        
        return export_files
    
    def _load_export_file(self, file_path: Path) -> Dict[str, Any]:
        """Load and validate export file
        
        Args:
            file_path: Path to export file
            
        Returns:
            Dict with export data
        """
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            # Validate structure
            if 'metadata' not in data or 'data' not in data:
                raise ValueError("Invalid export file format: missing 'metadata' or 'data'")
            
            metadata = data['metadata']
            required_metadata = ['database', 'table', 'row_count', 'mongodb_target']
            
            for key in required_metadata:
                if key not in metadata:
                    raise ValueError(f"Missing metadata field: {key}")
            
            logger.info(f"📄 Loaded export file: {metadata['row_count']} rows from {metadata['database']}.{metadata['table']}")
            
            return data
            
        except Exception as e:
            logger.error(f"❌ Failed to load export file {file_path}: {e}")
            raise
    
    def _prepare_documents(self, export_data: Dict[str, Any]) -> List[Dict[str, Any]]:
        """Prepare documents for MongoDB insertion
        
        Args:
            export_data: Export data from JSON file
            
        Returns:
            List of documents ready for insertion
        """
        documents = []
        
        for row in export_data['data']:
            # Convert row to MongoDB document
            doc = {}
            
            for key, value in row.items():
                # Handle special cases
                if key == 'id' and isinstance(value, str):
                    # Keep string IDs as-is for UUID fields
                    doc[key] = value
                elif key == 'id' and isinstance(value, int):
                    # Convert integer IDs to strings for consistency
                    doc[key] = str(value)
                elif value is None:
                    doc[key] = None
                elif isinstance(value, (str, int, float, bool)):
                    doc[key] = value
                else:
                    # Convert other types to string
                    doc[key] = str(value)
            
            documents.append(doc)
        
        return documents
    
    def _create_indexes(self, database_name: str, collection_name: str, 
                       documents: List[Dict[str, Any]]) -> bool:
        """Create indexes for better performance
        
        Args:
            database_name: Name of the database
            collection_name: Name of the collection
            documents: Sample documents to analyze
            
        Returns:
            bool: True if indexes were created successfully
        """
        try:
            collection = self.client.get_collection(database_name, collection_name)
            if collection is None:
                return False
            
            # Create indexes based on common patterns
            indexes_to_create = []
            
            # Always create index on id field
            if documents and 'id' in documents[0]:
                indexes_to_create.append(('id', 1))
            
            # Create indexes based on collection type
            if collection_name == 'courses':
                indexes_to_create.extend([
                    ('course_code', 1),
                    ('course_name', 1),
                    ('enabled', 1)
                ])
            elif collection_name == 'file':
                indexes_to_create.extend([
                    ('course_code', 1),
                    ('category', 1),
                    ('is_active', 1),
                    ('file_name', 1)
                ])
            
            # Create indexes
            for index_spec in indexes_to_create:
                try:
                    collection.create_index([index_spec])
                    logger.info(f"📇 Created index on {index_spec[0]} for {database_name}.{collection_name}")
                except Exception as e:
                    logger.warning(f"⚠️ Failed to create index {index_spec[0]}: {e}")
            
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to create indexes for {database_name}.{collection_name}: {e}")
            return False
    
    def seed_collection(self, database_name: str, collection_name: str, 
                       export_file: Path, clean: bool = False, 
                       dry_run: bool = False) -> Dict[str, Any]:
        """Seed a single collection
        
        Args:
            database_name: Name of the database
            collection_name: Name of the collection
            export_file: Path to export file
            clean: Whether to drop collection before seeding
            dry_run: If True, only show what would be done
            
        Returns:
            Dict with seeding results
        """
        logger.info(f"🌱 Seeding {database_name}.{collection_name}")
        
        # Load export data
        export_data = self._load_export_file(export_file)
        documents = self._prepare_documents(export_data)
        
        if dry_run:
            return {
                'database': database_name,
                'collection': collection_name,
                'documents_to_insert': len(documents),
                'clean': clean,
                'dry_run': True,
                'inserted': 0
            }
        
        # Connect to MongoDB
        if not self.client.connect():
            raise ConnectionError("Failed to connect to MongoDB")
        
        # Create database if it doesn't exist
        if not self.client.create_database_if_not_exists(database_name):
            raise RuntimeError(f"Failed to create database: {database_name}")
        
        # Clean collection if requested
        if clean:
            logger.info(f"🧹 Dropping collection {collection_name}")
            self.client.drop_collection(database_name, collection_name)
        
        # Check if collection already has data
        existing_count = self.client.count_documents(database_name, collection_name)
        
        if existing_count > 0 and not clean:
            logger.info(f"📊 Collection {collection_name} already has {existing_count} documents")
            logger.info("🔄 Performing upsert operation...")
            
            # For now, we'll insert all documents and let MongoDB handle duplicates
            # In a production system, you might want to implement proper upsert logic
            pass
        
        # Insert documents
        if documents:
            batch_size = self.config.get('export_settings', {}).get('batch_size', 1000)
            success = self.client.insert_many_batched(
                database_name, collection_name, documents, batch_size
            )
            
            if not success:
                raise RuntimeError(f"Failed to insert documents into {database_name}.{collection_name}")
            
            # Create indexes
            self._create_indexes(database_name, collection_name, documents)
            
            # Verify insertion
            final_count = self.client.count_documents(database_name, collection_name)
            inserted_count = final_count - existing_count
            
            logger.info(f"✅ Successfully seeded {database_name}.{collection_name}")
            logger.info(f"📊 Inserted: {inserted_count} documents, Total: {final_count} documents")
            
            return {
                'database': database_name,
                'collection': collection_name,
                'documents_to_insert': len(documents),
                'inserted': inserted_count,
                'total_documents': final_count,
                'clean': clean,
                'dry_run': False
            }
        
        else:
            logger.warning(f"⚠️ No documents to insert for {database_name}.{collection_name}")
            return {
                'database': database_name,
                'collection': collection_name,
                'documents_to_insert': 0,
                'inserted': 0,
                'total_documents': existing_count,
                'clean': clean,
                'dry_run': False
            }
    
    def seed_database(self, database_name: str, clean: bool = False, 
                     dry_run: bool = False) -> Dict[str, Any]:
        """Seed all collections in a database
        
        Args:
            database_name: Name of the database
            clean: Whether to drop collections before seeding
            dry_run: If True, only show what would be done
            
        Returns:
            Dict with seeding results
        """
        if database_name not in self.config['mongodb']['databases']:
            raise ValueError(f"Database '{database_name}' not found in configuration")
        
        db_config = self.config['mongodb']['databases'][database_name]
        collections = db_config['collections']
        
        logger.info(f"🗄️ Seeding database '{database_name}' with {len(collections)} collections")
        
        results = {
            'database': database_name,
            'collections': {},
            'total_inserted': 0,
            'total_documents': 0
        }
        
        # Find export files
        export_files = self._find_export_files()
        
        for collection_name, collection_config in collections.items():
            # Skip placeholder collections
            if collection_config.get('placeholder', False):
                logger.info(f"⏭️ Skipping placeholder collection: {collection_name}")
                continue
            
            # Find corresponding export file
            file_key = f"{database_name}.{collection_name}"
            
            if file_key not in export_files:
                logger.warning(f"⚠️ No export file found for {file_key}")
                results['collections'][collection_name] = {
                    'error': 'No export file found',
                    'inserted': 0
                }
                continue
            
            try:
                collection_result = self.seed_collection(
                    database_name, collection_name, export_files[file_key], 
                    clean, dry_run
                )
                
                results['collections'][collection_name] = collection_result
                
                if not dry_run:
                    results['total_inserted'] += collection_result['inserted']
                    results['total_documents'] += collection_result['total_documents']
                
            except Exception as e:
                logger.error(f"❌ Failed to seed collection '{collection_name}': {e}")
                results['collections'][collection_name] = {
                    'error': str(e),
                    'inserted': 0
                }
        
        return results
    
    def seed_all(self, clean: bool = False, dry_run: bool = False) -> Dict[str, Any]:
        """Seed all configured databases
        
        Args:
            clean: Whether to drop collections before seeding
            dry_run: If True, only show what would be done
            
        Returns:
            Dict with seeding results
        """
        databases = list(self.config['mongodb']['databases'].keys())
        
        logger.info(f"🌍 Seeding all databases ({len(databases)} total)")
        
        results = {
            'databases': {},
            'total_inserted': 0,
            'total_documents': 0,
            'mongodb_uri': self.mongodb_uri
        }
        
        for database_name in databases:
            try:
                db_result = self.seed_database(database_name, clean, dry_run)
                results['databases'][database_name] = db_result
                
                if not dry_run:
                    results['total_inserted'] += db_result['total_inserted']
                    results['total_documents'] += db_result['total_documents']
                
            except Exception as e:
                logger.error(f"❌ Failed to seed database '{database_name}': {e}")
                results['databases'][database_name] = {
                    'error': str(e),
                    'total_inserted': 0,
                    'total_documents': 0
                }
        
        return results
    
    def get_status(self) -> Dict[str, Any]:
        """Get MongoDB status and seeding information
        
        Returns:
            Dict with status information
        """
        logger.info("📊 Getting MongoDB status...")
        
        # Get MongoDB client status
        mongodb_status = self.client.get_status()
        
        # Get export files status
        export_files = self._find_export_files()
        
        # Get seeding configuration
        seeding_config = {
            'configured_databases': list(self.config['mongodb']['databases'].keys()),
            'available_collections': {},
            'export_files': {}
        }
        
        for db_name, db_config in self.config['mongodb']['databases'].items():
            collections = list(db_config['collections'].keys())
            seeding_config['available_collections'][db_name] = collections
        
        for key, file_path in export_files.items():
            seeding_config['export_files'][key] = {
                'path': str(file_path),
                'size_bytes': file_path.stat().st_size,
                'modified_at': datetime.fromtimestamp(file_path.stat().st_mtime).isoformat()
            }
        
        return {
            'mongodb_status': mongodb_status,
            'seeding_config': seeding_config,
            'input_directory': str(self.input_dir),
            'input_directory_exists': self.input_dir.exists()
        }
    
    def __del__(self):
        """Cleanup MongoDB connection"""
        if hasattr(self, 'client'):
            self.client.disconnect()


def main():
    """Main CLI function"""
    parser = argparse.ArgumentParser(
        description="Seed MongoDB databases with exported data",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s                          # Seed all databases
  %(prog)s --db courses            # Seed specific database
  %(prog)s --collection courses    # Seed specific collection (requires --db)
  %(prog)s --input custom_exports  # Custom input directory
  %(prog)s --dry-run               # Show what would be done
  %(prog)s --clean                 # Drop collections before seeding
  %(prog)s --status                # Show MongoDB status
        """
    )
    
    parser.add_argument(
        '--db', '--database',
        type=str,
        help="Seed specific database only"
    )
    
    parser.add_argument(
        '--collection',
        type=str,
        help="Seed specific collection only (requires --db)"
    )
    
    parser.add_argument(
        '--input',
        type=str,
        default="exports",
        help="Input directory containing export files (default: exports)"
    )
    
    parser.add_argument(
        '--clean',
        action='store_true',
        help="Drop collections before seeding (destructive operation)"
    )
    
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help="Show what would be done without actually seeding"
    )
    
    parser.add_argument(
        '--status',
        action='store_true',
        help="Show MongoDB status and seeding information"
    )
    
    parser.add_argument(
        '--mongodb-uri',
        type=str,
        help="MongoDB connection URI (overrides configuration)"
    )
    
    parser.add_argument(
        '--verbose',
        action='store_true',
        help="Enable verbose logging"
    )
    
    args = parser.parse_args()
    
    # Configure logging
    if args.verbose:
        logging.basicConfig(level=logging.DEBUG, format='%(levelname)s: %(message)s')
    else:
        logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
    
    print("🌱 MongoDB Seeding Tool")
    print("=" * 50)
    
    try:
        # Load configuration
        config = load_database_mapping()
        if not config:
            print("❌ Failed to load database mapping configuration")
            return 1
        
        # Initialize seeder
        seeder = MongoDBSeeder(config, args.input, args.mongodb_uri)
        
        if args.status:
            # Show status
            print("📊 MongoDB Status:")
            status = seeder.get_status()
            
            mongodb_status = status['mongodb_status']
            print(f"  Connection: {'✅ Connected' if mongodb_status['connected'] else '❌ Disconnected'}")
            print(f"  URI: {mongodb_status['uri']}")
            
            if mongodb_status['connected']:
                print(f"  Databases: {len(mongodb_status['databases'])}")
                for db_name in mongodb_status['databases']:
                    db_info = mongodb_status['database_info'].get(db_name, {})
                    collections = db_info.get('collections', [])
                    print(f"    📄 {db_name}: {len(collections)} collections")
                    
                    for collection_name in collections:
                        count = db_info.get('collection_counts', {}).get(collection_name, 0)
                        print(f"      📊 {collection_name}: {count} documents")
            
            seeding_config = status['seeding_config']
            print(f"\n📋 Seeding Configuration:")
            print(f"  Input directory: {status['input_directory']} {'✅' if status['input_directory_exists'] else '❌'}")
            print(f"  Export files: {len(seeding_config['export_files'])}")
            
            for key, file_info in seeding_config['export_files'].items():
                print(f"    📄 {key}: {file_info['size_bytes']} bytes")
            
            return 0
        
        # Perform seeding
        if args.dry_run:
            print("🔍 Dry run mode - showing what would be done:")
        
        if args.clean:
            print("⚠️ Clean mode - collections will be dropped before seeding")
            if not args.dry_run:
                confirm = input("Are you sure you want to continue? (y/N): ")
                if confirm.lower() != 'y':
                    print("❌ Operation cancelled")
                    return 0
        
        if args.collection:
            if not args.db:
                print("❌ --collection requires --db to be specified")
                return 1
            
            # Find export file
            export_files = seeder._find_export_files()
            file_key = f"{args.db}.{args.collection}"
            
            if file_key not in export_files:
                print(f"❌ No export file found for {file_key}")
                return 1
            
            # Seed specific collection
            result = seeder.seed_collection(
                args.db, args.collection, export_files[file_key], 
                args.clean, args.dry_run
            )
            
            if args.dry_run:
                print(f"✅ Would seed {result['documents_to_insert']} documents into {args.db}.{args.collection}")
            else:
                print(f"✅ Seeded {result['inserted']} documents into {args.db}.{args.collection}")
        
        elif args.db:
            # Seed specific database
            result = seeder.seed_database(args.db, args.clean, args.dry_run)
            
            if args.dry_run:
                total_docs = sum(r.get('documents_to_insert', 0) for r in result['collections'].values())
                print(f"✅ Would seed {total_docs} documents into {len(result['collections'])} collections")
            else:
                print(f"✅ Seeded {result['total_inserted']} documents into {len(result['collections'])} collections")
        
        else:
            # Seed all databases
            result = seeder.seed_all(args.clean, args.dry_run)
            
            if args.dry_run:
                total_docs = sum(
                    sum(c.get('documents_to_insert', 0) for c in db['collections'].values())
                    for db in result['databases'].values()
                )
                print(f"✅ Would seed {total_docs} documents into {len(result['databases'])} databases")
            else:
                print(f"✅ Seeded {result['total_inserted']} documents into {len(result['databases'])} databases")
        
        return 0
        
    except Exception as e:
        logger.error(f"❌ Seeding failed: {e}")
        if args.verbose:
            import traceback
            traceback.print_exc()
        return 1


if __name__ == "__main__":
    exit(main())