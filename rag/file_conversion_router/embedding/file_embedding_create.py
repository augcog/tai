import sqlite3
import numpy as np
from pathlib import Path
from typing import Optional, Dict, Any
from sentence_transformers import SentenceTransformer
from tqdm import tqdm


def _to_blob(vec: np.ndarray) -> bytes:
    """Convert numpy array to bytes for database storage"""
    return np.asarray(vec, dtype=np.float32).tobytes()


def _ensure_vector_column(conn: sqlite3.Connection):
    """Add vector column to file table if it doesn't exist"""
    try:
        cols = {row[1] for row in conn.execute("PRAGMA table_info(file)")}
        if "vector" not in cols:
            conn.execute("ALTER TABLE file ADD COLUMN vector BLOB")
            conn.commit()
            print("Added 'vector' column to file table")
    except sqlite3.OperationalError as e:
        print(f"Error adding vector column: {e}")
        raise


def read_markdown_file(file_path: Path) -> str:
    """Read markdown file content"""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read().strip()
        return content
    except Exception as e:
        print(f"Error reading {file_path}: {e}")
        return ""


def embed_files_from_markdown(
    db_path: str,
    data_dir: str,
    embedding_model: str = "Qwen/Qwen3-Embedding-4B",
    course_filter: Optional[str] = None,
    force_recompute: bool = False
) -> Dict[str, Any]:
    """
    Embed files by reading their corresponding markdown files and save vectors in file table.
    
    Args:
        db_path: Path to SQLite database
        data_dir: Base directory containing the markdown files
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
        # Ensure vector column exists
        _ensure_vector_column(conn)
        
        # Build query to get files
        params = []
        where_clauses = []
        
        if course_filter:
            where_clauses.append("(course_name = ? OR course_code = ?)")
            params.extend([course_filter, course_filter])
        
        # Only process files that don't have embeddings (unless force_recompute)
        if not force_recompute:
            where_clauses.append("vector IS NULL")
        
        file_query = """
            SELECT uuid, file_name, relative_path, course_name, course_code
            FROM file
        """
        
        if where_clauses:
            file_query += " WHERE " + " AND ".join(where_clauses)
        
        files = conn.execute(file_query, params).fetchall()
        
        if not files:
            return {
                "status": "no_files_found",
                "message": "No files found matching the criteria or all files already have embeddings",
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
            uuid = file_record["uuid"]
            file_name = file_record["file_name"]
            relative_path = file_record["relative_path"]
            course_name = file_record["course_name"] or ""
            course_code = file_record["course_code"] or ""
            
            # Construct markdown file path
            # Assuming the markdown file has the same name but with .md extension
            # and is in the same relative directory structure
            original_path = Path(relative_path)
            markdown_file_name = original_path.name + ".md"
            markdown_relative_path = original_path.parent /original_path.stem / markdown_file_name
            markdown_full_path = data_dir / markdown_relative_path
            
            # Try alternative markdown paths if the direct one doesn't exist
            if not markdown_full_path.exists():
                # Try looking in a markdown output directory structure
                # Common patterns: file.ext -> file/file.md or file.ext -> file.md
                alt_paths = [
                    data_dir / original_path.parent / original_path.stem / f"{original_path.stem}.md",
                    data_dir / f"{original_path.stem}.md",
                    data_dir / relative_path.replace(original_path.suffix, ".md") if original_path.suffix else None
                ]
                
                found_path = None
                for alt_path in alt_paths:
                    if alt_path and alt_path.exists():
                        found_path = alt_path
                        break
                
                if found_path:
                    markdown_full_path = found_path
                else:
                    print(f"Markdown file not found for {file_name}. Tried: {markdown_full_path}")
                    error_files += 1
                    results.append({
                        "uuid": uuid,
                        "file_name": file_name,
                        "status": "error",
                        "error": "Markdown file not found"
                    })
                    continue
            
            # Read markdown content
            markdown_content = read_markdown_file(markdown_full_path)
            
            if not markdown_content:
                print(f"Empty or unreadable markdown file: {markdown_full_path}")
                error_files += 1
                results.append({
                    "uuid": uuid,
                    "file_name": file_name,
                    "status": "error",
                    "error": "Empty or unreadable markdown file"
                })
                continue
            
            try:
                # Create embedding text with metadata context
                embedding_text = f"File: {file_name}\nCourse: {course_name} ({course_code})\n\nContent:\n{markdown_content}"
                
                # Generate embedding
                embedding = model.encode([embedding_text])[0]
                embedding_blob = _to_blob(embedding)
                
                # Update file table with embedding
                conn.execute(
                    "UPDATE file SET vector = ? WHERE uuid = ?",
                    (embedding_blob, uuid)
                )
                
                processed_files += 1
                results.append({
                    "uuid": uuid,
                    "file_name": file_name,
                    "markdown_path": str(markdown_full_path.relative_to(data_dir)),
                    "content_length": len(markdown_content),
                    "status": "success"
                })
                
            except Exception as e:
                print(f"Error processing file {file_name}: {e}")
                error_files += 1
                results.append({
                    "uuid": uuid,
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


def get_file_vector(db_path: str, uuid: str) -> Optional[np.ndarray]:
    """
    Retrieve file embedding vector by UUID
    
    Args:
        db_path: Path to SQLite database
        uuid: File UUID to lookup
        
    Returns:
        Numpy array of embedding or None if not found
    """
    conn = sqlite3.connect(db_path)
    try:
        result = conn.execute(
            "SELECT vector FROM file WHERE uuid = ?",
            (uuid,)
        ).fetchone()
        
        if result and result[0]:
            # Convert bytes back to numpy array
            return np.frombuffer(result[0], dtype=np.float32)
        return None
        
    finally:
        conn.close()


def check_embedding_status(db_path: str, course_filter: Optional[str] = None) -> Dict[str, Any]:
    """
    Check how many files have embeddings vs need embeddings
    
    Args:
        db_path: Path to SQLite database
        course_filter: Optional course name/code filter
        
    Returns:
        Dictionary with embedding statistics
    """
    conn = sqlite3.connect(db_path)
    try:
        params = []
        where_clause = ""
        
        if course_filter:
            where_clause = "WHERE (course_name = ? OR course_code = ?)"
            params.extend([course_filter, course_filter])
        
        # Count total files
        total_query = f"SELECT COUNT(*) FROM file {where_clause}"
        total_files = conn.execute(total_query, params).fetchone()[0]
        
        # Count files with embeddings (check in chunks table)
        if course_filter:
            embedded_query = """
                SELECT COUNT(DISTINCT f.uuid) 
                FROM file f
                INNER JOIN chunks c ON f.uuid = c.file_uuid
                WHERE (f.course_name = ? OR f.course_code = ?)
                AND c.vector IS NOT NULL
            """
            embedded_files = conn.execute(embedded_query, params).fetchone()[0]
        else:
            embedded_query = """
                SELECT COUNT(DISTINCT f.uuid) 
                FROM file f
                INNER JOIN chunks c ON f.uuid = c.file_uuid
                WHERE c.vector IS NOT NULL
            """
            embedded_files = conn.execute(embedded_query).fetchone()[0]
        
        return {
            "total_files": total_files,
            "embedded_files": embedded_files,
            "pending_files": total_files - embedded_files,
            "completion_percentage": (embedded_files / total_files * 100) if total_files > 0 else 0
        }
        
    finally:
        conn.close()


if __name__ == "__main__":
    # Example usage
    db_path = "/home/bot/bot/yk/YK_final/course_yaml/metadata.db"
    data_dir = "/home/bot/bot/yk/YK_final/courses_out"
    
    # Check current status
    status = check_embedding_status(db_path, course_filter="Berkeley")
    print("Current embedding status:", status)
    
    # Run embedding process
    results = embed_files_from_markdown(
        db_path=db_path,
        data_dir=data_dir,
        course_filter="Berkeley",  # Set to None to process all courses
        force_recompute=False  # Set to True to recompute existing embeddings
    )
    
    print("\nEmbedding results:")
    print(f"Processed: {results['processed']}")
    print(f"Errors: {results['errors']}")
    print(f"Status: {results['status']}")