"""
Memory Synopsis Service

This service handles MongoDB operations for memory synopsis storage and retrieval.
All operations include graceful error handling - failures don't break chat functionality.
"""

import logging
from datetime import datetime
from typing import Optional, List, Any
from uuid import uuid4

from app.core.mongodb_client import get_mongodb_client
from app.core.models.memory_synopsis import MemorySynopsisDocument
from app.core.models.chat_completion import Message
from app.services.rag_postprocess import MemorySynopsis, build_memory_synopsis

logger = logging.getLogger(__name__)


class MemorySynopsisService:
    """MongoDB service for memory synopsis operations with graceful error handling"""

    def __init__(self):
        self.mongodb_client = get_mongodb_client()
        self.database_name = "main"  # Temporary database name
        self.collection_name = "memory_synopses"

    async def get_by_chat_history_sid(self, chat_history_sid: str) -> Optional[MemorySynopsis]:
        """
        Retrieve memory synopsis by chat history SID.

        Args:
            chat_history_sid: The SID of the chat history from frontend

        Returns:
            MemorySynopsis object if found, None if not found or on error
        """
        try:
            collection = self.mongodb_client.get_collection(self.database_name, self.collection_name)
            if collection is None:
                print(f"[INFO] MongoDB collection not available, continuing without memory")
                return None

            # Find document by chat_history_sid
            document = collection.find_one({"chat_history_sid": chat_history_sid})

            if document is None:
                print(f"[INFO] No memory synopsis found for chat_history_sid: {chat_history_sid}")
                return None

            # Parse the content JSON back to MemorySynopsis
            memory_synopsis = MemorySynopsis.from_json(document["content"])
            print(f"[INFO] Retrieved memory synopsis for chat_history_sid: {chat_history_sid}")
            return memory_synopsis

        except Exception as e:
            print(f"[INFO] Failed to retrieve memory for SID {chat_history_sid}: {e}")
            return None  # Graceful degradation - return None if lookup fails

    async def create_or_update_memory(
        self,
        chat_history_sid: str,
        messages: List[Message],
        engine: Any,
        tokenizer: Any
    ) -> Optional[str]:
        """
        Create or update memory synopsis for a chat history.

        Args:
            chat_history_sid: The SID of the chat history from frontend
            messages: List of chat messages
            engine: LLM engine for memory generation
            tokenizer: Tokenizer for prompt processing

        Returns:
            memory_synopsis_sid if successful, None if failed
        """
        try:
            # Call build_memory_synopsis with chat_history_sid parameter
            # This will automatically retrieve previous memory from MongoDB
            new_memory = await build_memory_synopsis(
                messages=messages,
                tokenizer=tokenizer,
                engine=engine,
                chat_history_sid=chat_history_sid  # This triggers MongoDB lookup inside function
            )

            # Generate new memory synopsis SID
            memory_synopsis_sid = str(uuid4())

            # Get MongoDB collection
            collection = self.mongodb_client.get_collection(self.database_name, self.collection_name)
            if collection is None:
                print(f"[INFO] MongoDB collection not available, memory generation failed")
                return None

            # Upsert document using chat_history_sid as unique key
            result = collection.update_one(
                {"chat_history_sid": chat_history_sid},
                {
                    "$set": {
                        "memory_synopsis_sid": memory_synopsis_sid,
                        "content": new_memory.to_json(),
                        "updated_at": datetime.utcnow()
                    },
                    "$setOnInsert": {
                        "chat_history_sid": chat_history_sid,
                        "created_at": datetime.utcnow()
                    }
                },
                upsert=True
            )

            if result.upserted_id or result.modified_count > 0:
                print(f"[INFO] Successfully saved memory synopsis for chat_history_sid: {chat_history_sid}")
                return memory_synopsis_sid
            else:
                print(f"[INFO] Failed to save memory synopsis for chat_history_sid: {chat_history_sid}")
                return None

        except Exception as e:
            print(f"[INFO] Failed to create/update memory for SID {chat_history_sid}: {e}")
            return None  # Graceful failure - next conversation round will try again

    def delete_by_chat_history_sid(self, chat_history_sid: str) -> bool:
        """
        Delete memory synopsis by chat history SID.

        Args:
            chat_history_sid: The SID of the chat history from frontend

        Returns:
            True if deleted successfully, False otherwise
        """
        try:
            collection = self.mongodb_client.get_collection(self.database_name, self.collection_name)
            if collection is None:
                print(f"[INFO] MongoDB collection not available, delete failed")
                return False

            result = collection.delete_one({"chat_history_sid": chat_history_sid})

            if result.deleted_count > 0:
                print(f"[INFO] Successfully deleted memory synopsis for chat_history_sid: {chat_history_sid}")
                return True
            else:
                print(f"[INFO] No memory synopsis found to delete for chat_history_sid: {chat_history_sid}")
                return False

        except Exception as e:
            print(f"[INFO] Failed to delete memory for SID {chat_history_sid}: {e}")
            return False

    def get_status(self) -> dict:
        """
        Get service status and MongoDB collection information.

        Returns:
            Dict with service status
        """
        try:
            collection = self.mongodb_client.get_collection(self.database_name, self.collection_name)
            if collection is None:
                return {"status": "disconnected", "error": "MongoDB collection not available"}

            count = self.mongodb_client.count_documents(self.database_name, self.collection_name)

            return {
                "status": "connected",
                "database": self.database_name,
                "collection": self.collection_name,
                "document_count": count
            }

        except Exception as e:
            return {"status": "error", "error": str(e)}