import sqlite3
import json
import numpy as np
import hashlib
import os
from pathlib import Path
from typing import Optional, List, Dict, Any
from sentence_transformers import SentenceTransformer
from tqdm import tqdm


def compute_file_hash(file_path: str) -> str:
    """Compute SHA-256 hash of a file"""
    hash_sha256 = hashlib.sha256()
    try:
        with open(file_path, "rb") as f:
            for chunk in iter(lambda: f.read(4096), b""):
                hash_sha256.update(chunk)
        return hash_sha256.hexdigest()
    except Exception as e:
        print(f"Error computing hash for {file_path}: {e}")
        return ""


def _title_path_from_json(title_json: Optional[str]) -> str:
    """Extract title path from JSON string"""
    if not title_json:
        return ""
    try:
        arr = json.loads(title_json)
        if not isinstance(arr, (list, tuple)):
            arr = [str(arr)]
        parts = [str(x).strip() for x in arr if x is not None and str(x).strip()]
        return " > ".join(parts)
    except Exception:
        return str(title_json).strip()


def _to_blob(vec: np.ndarray) -> bytes:
    """Convert numpy array to bytes for database storage"""
    return np.asarray(vec, dtype=np.float32).tobytes()


def _ensure_embedding_tables(conn: sqlite3.Connection):
    """Ensure required tables exist for embeddings"""
    # Add chunks.vector if missing
    try:
        cols = {row[1] for row in conn.execute("PRAGMA table_info(chunks)")}
        if "vector" not in cols:
            conn.execute("ALTER TABLE chunks ADD COLUMN vector BLOB")
    except sqlite3.OperationalError:
        # chunks table doesn't exist, create it
        conn.execute("""
            CREATE TABLE IF NOT EXISTS chunks (
                chunk_uuid TEXT PRIMARY KEY,
                file_uuid TEXT,
                idx INTEGER,
                text TEXT,
                title TEXT,
                reference_path TEXT,
                file_path TEXT,
                course_name TEXT,
                course_code TEXT,
                vector BLOB
            )
        """)
    
    # Create file_embeddings table for storing file-level embeddings
    conn.execute("""
        CREATE TABLE IF NOT EXISTS file_embeddings (
            file_uuid TEXT PRIMARY KEY,
            file_hash TEXT UNIQUE,
            file_path TEXT,
            file_name TEXT,
            course_name TEXT,
            course_code TEXT,
            embedding_vector BLOB,
            embedding_model TEXT,
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
            updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
        )
    """)
    
    # Create index on file_hash for fast lookups
    conn.execute("CREATE INDEX IF NOT EXISTS idx_file_embeddings_hash ON file_embeddings(file_hash)")


def embed_all_files(
    db_path: str,
    data_dir: str,
    embedding_model: str = "Qwen/Qwen3-Embedding-4B",
    course_filter: Optional[str] = None,
    force_recompute: bool = False
) -> Dict[str, Any]:
    """
    Embed all files in the database and save results indexed by file hash/UUID.
    
    Args:
        db_path: Path to SQLite database
        data_dir: Base directory containing the files
        embedding_model: Name of the embedding model to use
        course_filter: Optional course name/code filter
        force_recompute: If True, recompute embeddings even if they exist
        
    Returns:
        Dictionary with embedding results and statistics
    """
    db_path = Path(db_path)
    data_dir = Path(data_dir)
    
    if not db_path.exists():
        raise FileNotFoundError(f"Database not found: {db_path}")
    
    if not data_dir.exists():
        raise FileNotFoundError(f"Data directory not found: {data_dir}")
    
    # Connect to database
    conn = sqlite3.connect(str(db_path))
    conn.row_factory = sqlite3.Row
    
    try:
        # Ensure required tables exist
        _ensure_embedding_tables(conn)
        
        # Get all files from database
        params = []
        where_clauses = []
        
        if course_filter:
            where_clauses.append("(course_name = ? OR course_code = ?)")
            params.extend([course_filter, course_filter])
        
        file_query = """
            SELECT uuid, file_name, relative_path, course_name, course_code, url
            FROM file
        """
        
        if where_clauses:
            file_query += " WHERE " + " AND ".join(where_clauses)
        
        files = conn.execute(file_query, params).fetchall()
        
        if not files:
            return {
                "status": "no_files_found",
                "message": "No files found matching the criteria",
                "processed": 0,
                "errors": 0
            }
        
        print(f"Found {len(files)} files to process")
        
        # Initialize embedding model
        print(f"Loading embedding model: {embedding_model}")
        model = SentenceTransformer(
            embedding_model,
            model_kwargs={
                "attn_implementation": "flash_attention_2",
                "torch_dtype": "auto",
                "device_map": "auto",
            },
            tokenizer_kwargs={"padding_side": "left"},
        )
        
        processed_files = 0
        error_files = 0
        skipped_files = 0
        results = []
        
        # Process each file
        for file_record in tqdm(files, desc="Processing files"):
            file_uuid = file_record["uuid"]
            file_name = file_record["file_name"]
            relative_path = file_record["relative_path"]
            course_name = file_record["course_name"] or ""
            course_code = file_record["course_code"] or ""
            
            # Construct full file path
            full_file_path = data_dir / relative_path
            
            if not full_file_path.exists():
                print(f"File not found: {full_file_path}")
                error_files += 1
                continue
            
            # Compute file hash
            file_hash = compute_file_hash(str(full_file_path))
            if not file_hash:
                error_files += 1
                continue
            
            # Check if embedding already exists (unless force_recompute)
            if not force_recompute:
                existing = conn.execute(
                    "SELECT file_uuid FROM file_embeddings WHERE file_hash = ? OR file_uuid = ?",
                    (file_hash, file_uuid)
                ).fetchone()
                
                if existing:
                    skipped_files += 1
                    continue
            
            # Get chunks for this file to create file-level embedding
            chunks = conn.execute("""
                SELECT chunk_uuid, text, title, reference_path, file_path, idx
                FROM chunks 
                WHERE file_uuid = ?
                ORDER BY idx
            """, (file_uuid,)).fetchall()
            
            if not chunks:
                # If no chunks, create embedding from file metadata
                embedding_text = f"File: {file_name}\nCourse: {course_name} ({course_code})"
            else:
                # Combine all chunk texts for file-level embedding
                texts = []
                for chunk in chunks:
                    chunk_text = chunk["text"] or ""
                    title_path = _title_path_from_json(chunk["title"])
                    if title_path:
                        texts.append(f"{title_path}: {chunk_text}")
                    else:
                        texts.append(chunk_text)
                
                # Create a comprehensive text representation of the file
                embedding_text = f"File: {file_name}\nCourse: {course_name} ({course_code})\nContent:\n" + "\n\n".join(texts)
            
            try:
                # Generate embedding
                embedding = model.encode([embedding_text])[0]
                embedding_blob = _to_blob(embedding)
                
                # Store file embedding
                conn.execute("""
                    INSERT OR REPLACE INTO file_embeddings 
                    (file_uuid, file_hash, file_path, file_name, course_name, course_code, 
                     embedding_vector, embedding_model, updated_at)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?, CURRENT_TIMESTAMP)
                """, (
                    file_uuid, 
                    file_hash,
                    relative_path,
                    file_name,
                    course_name,
                    course_code,
                    embedding_blob,
                    embedding_model
                ))
                
                # Also update chunk embeddings if they exist
                if chunks:
                    chunk_texts = []
                    chunk_uuids = []
                    
                    for chunk in chunks:
                        chunk_text = chunk["text"] or ""
                        title_path = _title_path_from_json(chunk["title"])
                        prompt = f"document_hierarchy_path: {title_path}\ndocument: {chunk_text}\n"
                        
                        chunk_texts.append(prompt)
                        chunk_uuids.append(chunk["chunk_uuid"])
                    
                    # Generate chunk embeddings
                    if chunk_texts:
                        chunk_embeddings = model.encode(chunk_texts)
                        
                        # Update chunk embeddings
                        for i, chunk_uuid in enumerate(chunk_uuids):
                            chunk_embedding_blob = _to_blob(chunk_embeddings[i])
                            conn.execute(
                                "UPDATE chunks SET vector = ? WHERE chunk_uuid = ?",
                                (chunk_embedding_blob, chunk_uuid)
                            )
                
                processed_files += 1
                results.append({
                    "file_uuid": file_uuid,
                    "file_hash": file_hash,
                    "file_name": file_name,
                    "status": "success"
                })
                
            except Exception as e:
                print(f"Error processing file {file_name}: {e}")
                error_files += 1
                results.append({
                    "file_uuid": file_uuid,
                    "file_hash": file_hash,
                    "file_name": file_name,
                    "status": "error",
                    "error": str(e)
                })
        
        # Commit all changes
        conn.commit()
        
        return {
            "status": "completed",
            "total_files": len(files),
            "processed": processed_files,
            "skipped": skipped_files,
            "errors": error_files,
            "embedding_model": embedding_model,
            "results": results
        }
    
    finally:
        conn.close()


def get_file_embedding(db_path: str, file_uuid: Optional[str] = None, file_hash: Optional[str] = None) -> Optional[np.ndarray]:
    """
    Retrieve file embedding by UUID or hash
    
    Args:
        db_path: Path to SQLite database
        file_uuid: File UUID to lookup
        file_hash: File hash to lookup
        
    Returns:
        Numpy array of embedding or None if not found
    """
    if not file_uuid and not file_hash:
        raise ValueError("Must provide either file_uuid or file_hash")
    
    conn = sqlite3.connect(db_path)
    try:
        if file_uuid:
            result = conn.execute(
                "SELECT embedding_vector FROM file_embeddings WHERE file_uuid = ?",
                (file_uuid,)
            ).fetchone()
        else:
            result = conn.execute(
                "SELECT embedding_vector FROM file_embeddings WHERE file_hash = ?",
                (file_hash,)
            ).fetchone()
        
        if result and result[0]:
            # Convert bytes back to numpy array
            return np.frombuffer(result[0], dtype=np.float32)
        return None
        
    finally:
        conn.close()


if __name__ == "__main__":
    # Example usage
    db_path = "/home/bot/bot/yk/YK_final/courses/metadata.db"
    data_dir = "/home/bot/bot/yk/YK_final/courses"
    
    results = embed_all_files(
        db_path=db_path,
        data_dir=data_dir,
        course_filter="ROAR Academy",  # Optional: filter by course
        force_recompute=False  # Set to True to recompute existing embeddings
    )
    
    print(json.dumps(results, indent=2, default=str))