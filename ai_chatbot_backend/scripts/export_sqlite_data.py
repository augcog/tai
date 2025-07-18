#!/usr/bin/env python3
"""
Expandable SQLite Data Export Script

This script provides a flexible system for exporting data from SQLite databases
to JSON files for MongoDB seeding. It uses a configuration-driven approach
that makes it easy to add new databases and tables without code changes.

Usage:
    python scripts/export_sqlite_data.py              # Export all configured databases
    python scripts/export_sqlite_data.py --db courses.db  # Export specific database
    python scripts/export_sqlite_data.py --table courses  # Export specific table
    python scripts/export_sqlite_data.py --output custom_exports  # Custom output directory
    python scripts/export_sqlite_data.py --validate  # Validate configuration only
"""

import sqlite3
import json
import argparse
import logging
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Any, Optional, Set
import sys

# Add the parent directory to sys.path to import app modules
sys.path.append(str(Path(__file__).parent.parent))

from app.core.mongodb_client import load_database_mapping

logger = logging.getLogger(__name__)


class SQLiteDataExporter:
    """Expandable SQLite data export system"""
    
    def __init__(self, config: Dict[str, Any], output_dir: str = "exports"):
        """Initialize exporter with configuration
        
        Args:
            config: Database mapping configuration
            output_dir: Directory to save exported files
        """
        self.config = config
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        
        # Validate configuration
        self._validate_config()
    
    def _validate_config(self) -> bool:
        """Validate configuration structure
        
        Returns:
            bool: True if configuration is valid
        """
        required_keys = ['sqlite_databases', 'export_settings']
        
        for key in required_keys:
            if key not in self.config:
                raise ValueError(f"Missing required configuration key: {key}")
        
        # Validate SQLite database configurations
        for db_name, db_config in self.config['sqlite_databases'].items():
            if 'path' not in db_config:
                raise ValueError(f"Missing 'path' for database: {db_name}")
            
            if 'tables' not in db_config:
                raise ValueError(f"Missing 'tables' for database: {db_name}")
            
            # Validate table configurations
            for table_name, table_config in db_config['tables'].items():
                required_table_keys = ['primary_key', 'export_fields', 'mongodb_target']
                for key in required_table_keys:
                    if key not in table_config:
                        raise ValueError(f"Missing '{key}' for table {table_name} in database {db_name}")
        
        logger.info("✅ Configuration validation passed")
        return True
    
    def get_available_databases(self) -> List[str]:
        """Get list of available databases from configuration
        
        Returns:
            List of database names
        """
        return list(self.config['sqlite_databases'].keys())
    
    def get_available_tables(self, database_name: str = None) -> List[str]:
        """Get list of available tables
        
        Args:
            database_name: If provided, get tables for specific database
            
        Returns:
            List of table names
        """
        if database_name:
            if database_name not in self.config['sqlite_databases']:
                return []
            return list(self.config['sqlite_databases'][database_name]['tables'].keys())
        
        # Get all tables from all databases
        all_tables = []
        for db_config in self.config['sqlite_databases'].values():
            all_tables.extend(db_config['tables'].keys())
        
        return all_tables
    
    def _get_database_path(self, database_name: str) -> Path:
        """Get path to database file
        
        Args:
            database_name: Name of the database
            
        Returns:
            Path to database file
        """
        if database_name not in self.config['sqlite_databases']:
            raise ValueError(f"Database '{database_name}' not found in configuration")
        
        db_path = Path(self.config['sqlite_databases'][database_name]['path'])
        
        # If path is relative, make it relative to project root
        if not db_path.is_absolute():
            project_root = Path(__file__).parent.parent
            db_path = project_root / db_path
        
        return db_path
    
    def _introspect_table_schema(self, conn: sqlite3.Connection, table_name: str) -> Dict[str, Any]:
        """Introspect table schema from SQLite database
        
        Args:
            conn: SQLite connection
            table_name: Name of the table
            
        Returns:
            Dict with table schema information
        """
        cursor = conn.cursor()
        
        # Get table info
        cursor.execute(f"PRAGMA table_info({table_name})")
        columns = cursor.fetchall()
        
        schema = {
            'columns': {},
            'primary_keys': [],
            'row_count': 0
        }
        
        for col in columns:
            # col = (cid, name, type, notnull, dflt_value, pk)
            col_name = col[1]
            schema['columns'][col_name] = {
                'type': col[2],
                'not_null': bool(col[3]),
                'default': col[4],
                'primary_key': bool(col[5])
            }
            
            if col[5]:  # Primary key
                schema['primary_keys'].append(col_name)
        
        # Get row count
        cursor.execute(f"SELECT COUNT(*) FROM {table_name}")
        schema['row_count'] = cursor.fetchone()[0]
        
        return schema
    
    def _convert_sqlite_row_to_dict(self, row: sqlite3.Row, fields: List[str]) -> Dict[str, Any]:
        """Convert SQLite row to dictionary with type conversion
        
        Args:
            row: SQLite row object
            fields: List of fields to include
            
        Returns:
            Dict representation of the row
        """
        result = {}
        
        for field in fields:
            if field in row.keys():
                value = row[field]
                
                # Handle None values
                if value is None:
                    result[field] = None
                # Handle datetime strings
                elif isinstance(value, str) and ('T' in value or '-' in value):
                    try:
                        # Try to parse as datetime
                        datetime.fromisoformat(value.replace('Z', '+00:00'))
                        result[field] = value
                    except ValueError:
                        result[field] = value
                else:
                    result[field] = value
            else:
                logger.warning(f"⚠️ Field '{field}' not found in table, setting to None")
                result[field] = None
        
        return result
    
    def export_table(self, database_name: str, table_name: str, 
                    validate_only: bool = False) -> Dict[str, Any]:
        """Export a single table from SQLite database
        
        Args:
            database_name: Name of the database
            table_name: Name of the table
            validate_only: If True, only validate without exporting
            
        Returns:
            Dict with export results
        """
        if database_name not in self.config['sqlite_databases']:
            raise ValueError(f"Database '{database_name}' not found in configuration")
        
        db_config = self.config['sqlite_databases'][database_name]
        
        if table_name not in db_config['tables']:
            raise ValueError(f"Table '{table_name}' not found in database '{database_name}' configuration")
        
        table_config = db_config['tables'][table_name]
        db_path = self._get_database_path(database_name)
        
        if not db_path.exists():
            raise FileNotFoundError(f"Database file not found: {db_path}")
        
        logger.info(f"📊 Exporting table '{table_name}' from database '{database_name}'")
        
        # Connect to database
        conn = sqlite3.connect(str(db_path))
        conn.row_factory = sqlite3.Row  # Enable column access by name
        
        try:
            # Introspect table schema
            schema = self._introspect_table_schema(conn, table_name)
            logger.info(f"📋 Table schema: {len(schema['columns'])} columns, {schema['row_count']} rows")
            
            if validate_only:
                conn.close()
                return {
                    'database': database_name,
                    'table': table_name,
                    'schema': schema,
                    'config': table_config,
                    'validation': 'passed',
                    'exported': False
                }
            
            # Export data
            export_fields = table_config['export_fields']
            cursor = conn.cursor()
            
            # Build query with proper field quoting
            fields_str = ', '.join(f'"{field}"' for field in export_fields)
            query = f"SELECT {fields_str} FROM {table_name}"
            
            # Add ordering by primary key if available
            if table_config['primary_key'] in export_fields:
                query += f" ORDER BY \"{table_config['primary_key']}\""
            
            logger.info(f"🔍 Executing query: {query}")
            cursor.execute(query)
            
            # Fetch and convert data
            rows = []
            batch_size = self.config['export_settings'].get('batch_size', 1000)
            
            while True:
                batch = cursor.fetchmany(batch_size)
                if not batch:
                    break
                
                for row in batch:
                    row_dict = self._convert_sqlite_row_to_dict(row, export_fields)
                    rows.append(row_dict)
                
                if len(rows) % (batch_size * 5) == 0:  # Log every 5 batches
                    logger.info(f"📦 Processed {len(rows)} rows...")
            
            # Prepare export data
            export_data = {
                'metadata': {
                    'database': database_name,
                    'table': table_name,
                    'exported_at': datetime.now().isoformat(),
                    'row_count': len(rows),
                    'schema': schema,
                    'mongodb_target': table_config['mongodb_target']
                },
                'data': rows
            }
            
            # Save to file
            mongodb_target = table_config['mongodb_target']
            output_filename = f"{mongodb_target['database']}_{mongodb_target['collection']}.json"
            output_path = self.output_dir / output_filename
            
            with open(output_path, 'w', encoding='utf-8') as f:
                json.dump(export_data, f, indent=2, ensure_ascii=False)
            
            logger.info(f"💾 Exported {len(rows)} rows to {output_path}")
            
            return {
                'database': database_name,
                'table': table_name,
                'row_count': len(rows),
                'output_file': str(output_path),
                'mongodb_target': table_config['mongodb_target'],
                'exported': True
            }
            
        finally:
            conn.close()
    
    def export_database(self, database_name: str, validate_only: bool = False) -> Dict[str, Any]:
        """Export all tables from a database
        
        Args:
            database_name: Name of the database
            validate_only: If True, only validate without exporting
            
        Returns:
            Dict with export results
        """
        if database_name not in self.config['sqlite_databases']:
            raise ValueError(f"Database '{database_name}' not found in configuration")
        
        db_config = self.config['sqlite_databases'][database_name]
        tables = list(db_config['tables'].keys())
        
        logger.info(f"🗄️ Exporting database '{database_name}' with {len(tables)} tables")
        
        results = {
            'database': database_name,
            'tables': {},
            'total_rows': 0,
            'exported_files': []
        }
        
        for table_name in tables:
            try:
                table_result = self.export_table(database_name, table_name, validate_only)
                results['tables'][table_name] = table_result
                
                if table_result.get('exported', False):
                    results['total_rows'] += table_result['row_count']
                    results['exported_files'].append(table_result['output_file'])
                
            except Exception as e:
                logger.error(f"❌ Failed to export table '{table_name}': {e}")
                results['tables'][table_name] = {
                    'error': str(e),
                    'exported': False
                }
        
        return results
    
    def export_all(self, validate_only: bool = False) -> Dict[str, Any]:
        """Export all configured databases and tables
        
        Args:
            validate_only: If True, only validate without exporting
            
        Returns:
            Dict with export results
        """
        databases = self.get_available_databases()
        
        logger.info(f"🌍 Exporting all databases ({len(databases)} total)")
        
        results = {
            'databases': {},
            'total_rows': 0,
            'exported_files': [],
            'export_settings': self.config['export_settings']
        }
        
        for database_name in databases:
            try:
                db_result = self.export_database(database_name, validate_only)
                results['databases'][database_name] = db_result
                
                results['total_rows'] += db_result['total_rows']
                results['exported_files'].extend(db_result['exported_files'])
                
            except Exception as e:
                logger.error(f"❌ Failed to export database '{database_name}': {e}")
                results['databases'][database_name] = {
                    'error': str(e),
                    'exported': False
                }
        
        return results
    
    def get_export_summary(self) -> Dict[str, Any]:
        """Get summary of what would be exported
        
        Returns:
            Dict with export summary
        """
        summary = {
            'databases': {},
            'total_tables': 0,
            'estimated_total_rows': 0
        }
        
        for database_name in self.get_available_databases():
            try:
                db_path = self._get_database_path(database_name)
                if not db_path.exists():
                    logger.warning(f"⚠️ Database file not found: {db_path}")
                    continue
                
                conn = sqlite3.connect(str(db_path))
                db_config = self.config['sqlite_databases'][database_name]
                
                db_summary = {
                    'path': str(db_path),
                    'tables': {},
                    'total_rows': 0
                }
                
                for table_name in db_config['tables'].keys():
                    try:
                        schema = self._introspect_table_schema(conn, table_name)
                        db_summary['tables'][table_name] = {
                            'columns': len(schema['columns']),
                            'row_count': schema['row_count'],
                            'mongodb_target': db_config['tables'][table_name]['mongodb_target']
                        }
                        db_summary['total_rows'] += schema['row_count']
                        
                    except Exception as e:
                        logger.error(f"❌ Failed to introspect table '{table_name}': {e}")
                        db_summary['tables'][table_name] = {'error': str(e)}
                
                summary['databases'][database_name] = db_summary
                summary['total_tables'] += len(db_config['tables'])
                summary['estimated_total_rows'] += db_summary['total_rows']
                
                conn.close()
                
            except Exception as e:
                logger.error(f"❌ Failed to analyze database '{database_name}': {e}")
                summary['databases'][database_name] = {'error': str(e)}
        
        return summary


def main():
    """Main CLI function"""
    parser = argparse.ArgumentParser(
        description="Export SQLite data to JSON files for MongoDB seeding",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s                          # Export all databases
  %(prog)s --db courses.db         # Export specific database
  %(prog)s --table courses         # Export specific table
  %(prog)s --output custom_exports # Custom output directory
  %(prog)s --validate              # Validate configuration only
  %(prog)s --summary               # Show export summary
        """
    )
    
    parser.add_argument(
        '--db', '--database',
        type=str,
        help="Export specific database only"
    )
    
    parser.add_argument(
        '--table',
        type=str,
        help="Export specific table only (requires --db)"
    )
    
    parser.add_argument(
        '--output',
        type=str,
        default="exports",
        help="Output directory for exported files (default: exports)"
    )
    
    parser.add_argument(
        '--validate',
        action='store_true',
        help="Validate configuration and database structure without exporting"
    )
    
    parser.add_argument(
        '--summary',
        action='store_true',
        help="Show export summary without exporting"
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
    
    print("📊 SQLite Data Export Tool")
    print("=" * 50)
    
    try:
        # Load configuration
        config = load_database_mapping()
        if not config:
            print("❌ Failed to load database mapping configuration")
            return 1
        
        # Initialize exporter
        exporter = SQLiteDataExporter(config, args.output)
        
        if args.summary:
            # Show export summary
            print("📋 Export Summary:")
            summary = exporter.get_export_summary()
            
            for db_name, db_info in summary['databases'].items():
                if 'error' in db_info:
                    print(f"  ❌ {db_name}: {db_info['error']}")
                    continue
                
                print(f"  📄 {db_name}: {db_info['total_rows']} rows in {len(db_info['tables'])} tables")
                for table_name, table_info in db_info['tables'].items():
                    if 'error' in table_info:
                        print(f"    ❌ {table_name}: {table_info['error']}")
                    else:
                        target = table_info['mongodb_target']
                        print(f"    📊 {table_name}: {table_info['row_count']} rows → {target['database']}.{target['collection']}")
            
            print(f"\n📈 Total: {summary['estimated_total_rows']} rows across {summary['total_tables']} tables")
            return 0
        
        # Perform export
        if args.table:
            if not args.db:
                print("❌ --table requires --db to be specified")
                return 1
            
            # Export specific table
            result = exporter.export_table(args.db, args.table, args.validate)
            
            if args.validate:
                print(f"✅ Validation passed for table '{args.table}' in database '{args.db}'")
            else:
                print(f"✅ Exported {result['row_count']} rows to {result['output_file']}")
        
        elif args.db:
            # Export specific database
            result = exporter.export_database(args.db, args.validate)
            
            if args.validate:
                print(f"✅ Validation passed for database '{args.db}'")
            else:
                print(f"✅ Exported {result['total_rows']} rows from {len(result['tables'])} tables")
        
        else:
            # Export all databases
            result = exporter.export_all(args.validate)
            
            if args.validate:
                print("✅ Validation passed for all databases")
            else:
                print(f"✅ Exported {result['total_rows']} rows from {len(result['databases'])} databases")
                print(f"📁 Output files: {len(result['exported_files'])} files in {args.output}/")
        
        return 0
        
    except Exception as e:
        logger.error(f"❌ Export failed: {e}")
        if args.verbose:
            import traceback
            traceback.print_exc()
        return 1


if __name__ == "__main__":
    exit(main())