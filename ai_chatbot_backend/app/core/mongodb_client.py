"""
MongoDB Connection and Utilities

This module provides MongoDB connection management and utility functions
for interacting with the cloud MongoDB instance.
"""

import logging
from typing import Dict, List, Optional, Any
from pathlib import Path
import json

from pymongo import MongoClient
from pymongo.collection import Collection
from pymongo.database import Database
from pymongo.errors import ConnectionFailure, ServerSelectionTimeoutError

from app.config import settings

logger = logging.getLogger(__name__)


class MongoDBClient:
    """MongoDB client wrapper for cloud database operations"""
    
    def __init__(self, uri: str = None):
        """Initialize MongoDB client
        
        Args:
            uri: MongoDB connection URI. If None, uses settings.MONGODB_URI
        """
        self.uri = uri or settings.MONGODB_URI
        self.client: Optional[MongoClient] = None
        self._connection_tested = False
        
    def connect(self) -> bool:
        """Establish connection to MongoDB
        
        Returns:
            bool: True if connection successful, False otherwise
        """
        try:
            if self.client is None:
                logger.info("🔌 Connecting to MongoDB...")
                self.client = MongoClient(
                    self.uri,
                    serverSelectionTimeoutMS=5000,  # 5 second timeout
                    connectTimeoutMS=10000,         # 10 second timeout
                    socketTimeoutMS=20000,          # 20 second timeout
                    maxPoolSize=50,                 # Connection pool size
                    retryWrites=True
                )
                
            # Test connection
            if not self._connection_tested:
                self.client.admin.command('ping')
                self._connection_tested = True
                logger.info("✅ MongoDB connection successful")
                
            return True
            
        except (ConnectionFailure, ServerSelectionTimeoutError) as e:
            logger.error(f"❌ MongoDB connection failed: {e}")
            return False
        except Exception as e:
            logger.error(f"❌ Unexpected MongoDB connection error: {e}")
            return False
    
    def disconnect(self):
        """Close MongoDB connection"""
        if self.client:
            self.client.close()
            self.client = None
            self._connection_tested = False
            logger.info("📡 MongoDB connection closed")
    
    def get_database(self, database_name: str) -> Optional[Database]:
        """Get database instance
        
        Args:
            database_name: Name of the database
            
        Returns:
            Database instance or None if connection failed
        """
        if not self.connect():
            return None
            
        return self.client[database_name]
    
    def get_collection(self, database_name: str, collection_name: str) -> Optional[Collection]:
        """Get collection instance
        
        Args:
            database_name: Name of the database
            collection_name: Name of the collection
            
        Returns:
            Collection instance or None if connection failed
        """
        database = self.get_database(database_name)
        if database is None:
            return None
            
        return database[collection_name]
    
    def list_databases(self) -> List[str]:
        """List all databases
        
        Returns:
            List of database names
        """
        if not self.connect():
            return []
            
        try:
            db_info = self.client.list_database_names()
            return [db for db in db_info if db not in ['admin', 'local', 'config']]
        except Exception as e:
            logger.error(f"❌ Failed to list databases: {e}")
            return []
    
    def list_collections(self, database_name: str) -> List[str]:
        """List collections in a database
        
        Args:
            database_name: Name of the database
            
        Returns:
            List of collection names
        """
        database = self.get_database(database_name)
        if database is None:
            return []
            
        try:
            return database.list_collection_names()
        except Exception as e:
            logger.error(f"❌ Failed to list collections for database {database_name}: {e}")
            return []
    
    def create_database_if_not_exists(self, database_name: str) -> bool:
        """Create database if it doesn't exist
        
        Args:
            database_name: Name of the database to create
            
        Returns:
            bool: True if database exists or was created successfully
        """
        if not self.connect():
            return False
            
        try:
            # In MongoDB, databases are created automatically when you insert data
            # We'll just verify we can access it
            database = self.get_database(database_name)
            if database is not None:
                # Create a temporary collection to ensure database exists
                temp_collection = database['_temp_init']
                temp_collection.insert_one({'_temp': True})
                temp_collection.drop()
                logger.info(f"✅ Database '{database_name}' ready")
                return True
        except Exception as e:
            logger.error(f"❌ Failed to create database {database_name}: {e}")
            
        return False
    
    def drop_collection(self, database_name: str, collection_name: str) -> bool:
        """Drop a collection
        
        Args:
            database_name: Name of the database
            collection_name: Name of the collection to drop
            
        Returns:
            bool: True if collection was dropped successfully
        """
        collection = self.get_collection(database_name, collection_name)
        if collection is None:
            return False
            
        try:
            collection.drop()
            logger.info(f"🗑️ Dropped collection '{collection_name}' from database '{database_name}'")
            return True
        except Exception as e:
            logger.error(f"❌ Failed to drop collection {collection_name}: {e}")
            return False
    
    def insert_many_batched(self, database_name: str, collection_name: str, 
                           documents: List[Dict[str, Any]], batch_size: int = 1000) -> bool:
        """Insert many documents in batches
        
        Args:
            database_name: Name of the database
            collection_name: Name of the collection
            documents: List of documents to insert
            batch_size: Number of documents per batch
            
        Returns:
            bool: True if all documents were inserted successfully
        """
        collection = self.get_collection(database_name, collection_name)
        if collection is None:
            return False
            
        try:
            total_inserted = 0
            for i in range(0, len(documents), batch_size):
                batch = documents[i:i + batch_size]
                result = collection.insert_many(batch)
                total_inserted += len(result.inserted_ids)
                
                if total_inserted % (batch_size * 5) == 0:  # Log every 5 batches
                    logger.info(f"📦 Inserted {total_inserted}/{len(documents)} documents")
            
            logger.info(f"✅ Successfully inserted {total_inserted} documents into {database_name}.{collection_name}")
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to insert documents into {database_name}.{collection_name}: {e}")
            return False
    
    def count_documents(self, database_name: str, collection_name: str) -> int:
        """Count documents in a collection
        
        Args:
            database_name: Name of the database
            collection_name: Name of the collection
            
        Returns:
            int: Number of documents in the collection
        """
        collection = self.get_collection(database_name, collection_name)
        if collection is None:
            return 0
            
        try:
            return collection.count_documents({})
        except Exception as e:
            logger.error(f"❌ Failed to count documents in {database_name}.{collection_name}: {e}")
            return 0
    
    def find_all(self, database_name: str, collection_name: str) -> List[Dict[str, Any]]:
        """Find all documents in a collection
        
        Args:
            database_name: Name of the database
            collection_name: Name of the collection
            
        Returns:
            List of documents
        """
        collection = self.get_collection(database_name, collection_name)
        if collection is None:
            return []
            
        try:
            return list(collection.find({}))
        except Exception as e:
            logger.error(f"❌ Failed to find documents in {database_name}.{collection_name}: {e}")
            return []
    
    def get_status(self) -> Dict[str, Any]:
        """Get MongoDB connection status and database information
        
        Returns:
            Dict with connection status and database info
        """
        status = {
            'connected': False,
            'uri': self.uri,
            'databases': [],
            'error': None
        }
        
        try:
            if self.connect():
                status['connected'] = True
                status['databases'] = self.list_databases()
                
                # Get database sizes and collection counts
                database_info = {}
                for db_name in status['databases']:
                    collections = self.list_collections(db_name)
                    database_info[db_name] = {
                        'collections': collections,
                        'collection_counts': {}
                    }
                    
                    for collection_name in collections:
                        count = self.count_documents(db_name, collection_name)
                        database_info[db_name]['collection_counts'][collection_name] = count
                
                status['database_info'] = database_info
                
        except Exception as e:
            status['error'] = str(e)
            
        return status


def load_database_mapping() -> Dict[str, Any]:
    """Load database mapping configuration
    
    Returns:
        Dict with database mapping configuration
    """
    mapping_path = Path("config/database_mapping.json")
    
    if not mapping_path.exists():
        logger.error(f"❌ Database mapping file not found: {mapping_path}")
        return {}
        
    try:
        with open(mapping_path, 'r', encoding='utf-8') as f:
            config = json.load(f)
            
        # Replace ENV references with actual environment variables
        if 'mongodb' in config and 'uri' in config['mongodb']:
            uri = config['mongodb']['uri']
            if uri.startswith('ENV:'):
                env_var = uri[4:]  # Remove 'ENV:' prefix
                from app.config import settings
                config['mongodb']['uri'] = getattr(settings, env_var, uri)
                
        return config
    except Exception as e:
        logger.error(f"❌ Failed to load database mapping: {e}")
        return {}


# Global MongoDB client instance
_mongodb_client = None


def get_mongodb_client() -> MongoDBClient:
    """Get global MongoDB client instance
    
    Returns:
        MongoDBClient instance
    """
    global _mongodb_client
    if _mongodb_client is None:
        _mongodb_client = MongoDBClient()
    return _mongodb_client


def test_mongodb_connection() -> bool:
    """Test MongoDB connection
    
    Returns:
        bool: True if connection successful
    """
    client = get_mongodb_client()
    return client.connect()