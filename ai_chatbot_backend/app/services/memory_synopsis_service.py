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
from app.services.rag_postprocess import MemorySynopsis, build_memory_synopsis, MemorySynopsisLong, build_memory_synopsis_long
from fastapi.responses import JSONResponse
from app.services.rag_generation import format_chat_msg
from app.dependencies.model import get_model_engine


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


"""
Memory Synopsis Long-Term Service

This service handles MongoDB operations for Long-Term Memory (LTM) storage and retrieval,
linking the LTM to a specific user_id.
"""


logger = logging.getLogger(__name__)


class MemorySynopsisServiceLong:
    """MongoDB service for Long-Term Memory (LTM) operations, linked to user_id."""

    def __init__(self):
        self.mongodb_client = get_mongodb_client()
        self.database_name = "main"  # 保持与 STM 服务一致
        self.collection_name = "long_term_memories" # LTM 专用集合名

    async def get_by_user_id(self, user_id: str) -> Optional[MemorySynopsisLong]:
        """
        Retrieve Long-Term Memory (LTM) by user ID.

        Args:
            user_id: The unique ID of the user.

        Returns:
            MemorySynopsisLong object if found, None if not found or on error.
        """
        try:
            collection = self.mongodb_client.get_collection(self.database_name, self.collection_name)
            if collection is None:
                print(f"[INFO] MongoDB LTM collection not available, continuing without LTM")
                return None

            # Find document by user_id
            document = collection.find_one({"user_id": user_id})

            if document is None:
                print(f"[INFO] No LTM found for user_id: {user_id}")
                return None

            # Parse the content JSON back to MemorySynopsisLong
            memory_synopsis_long = MemorySynopsisLong.from_json(document["content"])
            print(f"[INFO] Retrieved LTM for user_id: {user_id}")
            return memory_synopsis_long

        except Exception as e:
            print(f"[INFO] Failed to retrieve LTM for user ID {user_id}: {e}")
            return None  # Graceful degradation - return None if lookup fails

    async def create_or_update_ltm(
        self,
        user_id: str,
        chat_history_sid: str, # 保持与会话的关联，方便调试/溯源
        messages: List[Message],
        new_stm: MemorySynopsis,
        engine: Any,
        tokenizer: Any
    ) -> Optional[str]:
        """
        Create or update Long-Term Memory (LTM) for a user.

        Args:
            user_id: The unique ID of the user.
            chat_history_sid: The current session's SID (for context/debugging).
            messages: List of chat messages from the current session.
            new_stm: The newly generated Short-Term Memory for the current session.
            engine: LLM engine for LTM synthesis.
            tokenizer: Tokenizer for prompt processing.

        Returns:
            ltm_synopsis_sid if successful, None if failed.
        """
        try:
            # 1. 尝试获取之前的 LTM
            prev_ltm = await self.get_by_user_id(user_id)

            # 2. 调用 build_memory_synopsis_long 生成新的 LTM
            new_ltm = await build_memory_synopsis_long(
                messages=messages,
                tokenizer=tokenizer,
                engine=engine,
                new_stm=new_stm,
                prev_synopsis_long=prev_ltm,
                chat_history_sid=chat_history_sid # 传递 sid，尽管 LTM 是以 user_id 检索的
            )

            # 3. 生成新的 LTM SID
            ltm_synopsis_sid = str(uuid4())

            # 4. 获取 MongoDB collection
            collection = self.mongodb_client.get_collection(self.database_name, self.collection_name)
            if collection is None:
                print(f"[INFO] MongoDB LTM collection not available, LTM generation failed")
                return None

            # 5. Upsert document using user_id as unique key
            result = collection.update_one(
                {"user_id": user_id},
                {
                    "$set": {
                        "ltm_synopsis_sid": ltm_synopsis_sid,
                        "content": new_ltm.to_json(),
                        "updated_at": datetime.utcnow(),
                        "last_chat_sid": chat_history_sid # 记录最后一次更新 LTM 的会话 SID
                    },
                    "$setOnInsert": {
                        "user_id": user_id,
                        "created_at": datetime.utcnow()
                    }
                },
                upsert=True
            )

            if result.upserted_id or result.modified_count > 0:
                print(f"[INFO] Successfully saved LTM for user_id: {user_id}")
                return ltm_synopsis_sid
            else:
                print(f"[INFO] Failed to save LTM for user_id: {user_id}")
                return None

        except Exception as e:
            print(f"[INFO] Failed to create/update LTM for user ID {user_id}: {e}")
            return None

    def delete_by_user_id(self, user_id: str) -> bool:
        """
        Delete Long-Term Memory (LTM) by user ID.

        Args:
            user_id: The unique ID of the user.

        Returns:
            True if deleted successfully, False otherwise
        """
        try:
            collection = self.mongodb_client.get_collection(self.database_name, self.collection_name)
            if collection is None:
                print(f"[INFO] MongoDB LTM collection not available, delete failed")
                return False

            result = collection.delete_one({"user_id": user_id})

            if result.deleted_count > 0:
                print(f"[INFO] Successfully deleted LTM for user_id: {user_id}")
                return True
            else:
                print(f"[INFO] No LTM found to delete for user_id: {user_id}")
                return False

        except Exception as e:
            print(f"[INFO] Failed to delete LTM for user ID {user_id}: {e}")
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

            # 假设 mongodb_client 有 count_documents 方法
            count = self.mongodb_client.count_documents(self.database_name, self.collection_name)

            return {
                "status": "connected",
                "database": self.database_name,
                "collection": self.collection_name,
                "document_count": count
            }

        except Exception as e:
            return {"status": "error", "error": str(e)}

async def create_or_update_memory_synopsis(
        sid: str,
        user_id: str, # 假设已在 GeneralCompletionParams, FileCompletionParams, PracticeCompletionParams 中添加
        messages: List[Message],
        # course_code: str,
        # _: bool = Depends(verify_api_token)
):
    """
    Create or update memory synopsis (STM) for a chat history, and then synthesize
    and update Long-Term Memory (LTM) for the user.
    """
    logger.info(f"[INFO] Memory Request received. SID: {sid}")
    print(f"[INFO] Memory Request received. SID: {sid}")
    try:
        # Get the pre-initialized pipeline
        engine = get_model_engine()
        print(f"[INFO] Using engine: {engine}")

        # Import TOKENIZER from rag_generation
        from app.services.rag_generation import TOKENIZER

        # Initialize memory synopsis service
        stm_service = MemorySynopsisService()
        print("[INFO] STM Service initialized.")
        ltm_service = MemorySynopsisServiceLong()
        print("[INFO] LTM Service initialized.")
        formatted_messages = format_chat_msg(messages)
        print("[INFO] Messages formatted for memory synopsis.")

        # 1. 步骤 a: 生成 Short-Term Memory (STM) 实例
        # 这一步是为了获取 MemorySynopsis 实例 (new_stm) 供 LTM 使用
        new_memory: MemorySynopsis = await build_memory_synopsis(
            messages=formatted_messages,
            tokenizer=TOKENIZER,
            engine=engine,
            chat_history_sid=sid # build_memory_synopsis 会自动检索旧的 STM
        )
        print("[INFO] Short-Term Memory (STM) instance built.")
        # 2. 步骤 b: 存储/更新 Short-Term Memory (STM)
        # 调用 STM 服务进行持久化。STM 服务内部会再次调用 build_memory_synopsis
        memory_synopsis_sid = await stm_service.create_or_update_memory(
            chat_history_sid=sid,
            messages=formatted_messages,
            engine=engine,
            tokenizer=TOKENIZER
        )
        print("[INFO] Short-Term Memory (STM) stored/updated.")
        ltm_synopsis_sid = None
        # 3. 步骤 c: 如果 STM 成功，生成并存储 Long-Term Memory (LTM)
        if memory_synopsis_sid:
            ltm_synopsis_sid = await ltm_service.create_or_update_ltm(
                user_id=user_id,
                chat_history_sid=sid,
                messages=formatted_messages,
                new_stm=new_memory, # <--- 修正：传递第一步生成的 MemorySynopsis 实例
                engine=engine,
                tokenizer=TOKENIZER
            )
        print("[INFO] Long-Term Memory (LTM) stored/updated.")
        # 4. 返回结果
        if memory_synopsis_sid:
            return JSONResponse({
                "memory_synopsis_sid": memory_synopsis_sid, # STM SID
                "ltm_synopsis_sid": ltm_synopsis_sid,       # LTM SID
                "status": "success",
                "message": "Memory synopses (STM and LTM) created/updated successfully"
            })
        else:
            return JSONResponse({
                "status": "failed",
                "message": "Memory generation failed, will retry next round"
            })

    except Exception as e:
        print(f"[INFO] Memory synopsis endpoint failed: {e}")
        return JSONResponse({
            "status": "failed",
            "message": "Memory generation failed due to internal error, will retry next round"
        })
