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

        # Try to use FlashAttention2 if available, otherwise use default
        try:
            import flash_attn
            model_kwargs = {
                "attn_implementation": "flash_attention_2",
                "torch_dtype": "auto",
                "device_map": "auto",
            }
            print("Using FlashAttention2 for faster inference")
        except ImportError:
            model_kwargs = {
                "torch_dtype": "auto",
                "device_map": "auto",
            }
            print("FlashAttention2 not available, using default attention implementation")

        model = SentenceTransformer(
            embedding_model,
            model_kwargs=model_kwargs,
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
            # MinerU creates: input_dir/subdir/file.ext -> output_dir/subdir/file/file.ext.md
            # Note: relative_path is stored relative to parent of input_dir, so first component needs stripping
            original_path = Path(relative_path)

            # Strip first path component (the base directory name)
            # relative_path is always "base_dir/subdir/file.ext" but output_dir already includes base_dir
            path_parts = list(original_path.parts)
            if len(path_parts) > 1:
                working_path = Path(*path_parts[1:])
            else:
                working_path = original_path

            markdown_file_name = working_path.name + ".md"
            markdown_relative_path = working_path.parent / working_path.stem / markdown_file_name
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
    Check embedding status for both files and chunks

    Args:
        db_path: Path to SQLite database
        course_filter: Optional course name/code filter

    Returns:
        Dictionary with comprehensive embedding statistics:
            - total_files: Total number of files
            - files_embedded: Files with file.vector populated
            - files_pending: Files without file.vector
            - total_chunks: Total number of chunks
            - chunks_embedded: Chunks with chunks.vector populated
            - chunks_pending: Chunks without chunks.vector
            - files_completion_percentage: % of files with embeddings
            - chunks_completion_percentage: % of chunks with embeddings
            - total_embedded: Combined count (for backward compatibility)
    """
    conn = sqlite3.connect(db_path)
    try:
        # Ensure vector column exists before checking status
        _ensure_vector_column(conn)

        params = []
        where_clause = ""

        if course_filter:
            where_clause = "WHERE (course_name = ? OR course_code = ?)"
            params.extend([course_filter, course_filter])

        # === FILE-LEVEL EMBEDDINGS ===
        # Count total files
        total_files_query = f"SELECT COUNT(*) FROM file {where_clause}"
        total_files = conn.execute(total_files_query, params).fetchone()[0]

        # Count files with embeddings (file.vector is populated)
        files_embedded_query = f"""
            SELECT COUNT(*)
            FROM file
            {where_clause} {'AND' if where_clause else 'WHERE'} vector IS NOT NULL AND vector != ''
        """
        files_embedded = conn.execute(files_embedded_query, params if course_filter else []).fetchone()[0]
        files_pending = total_files - files_embedded

        # === CHUNK-LEVEL EMBEDDINGS ===
        # Count total chunks
        if course_filter:
            total_chunks_query = """
                SELECT COUNT(*) FROM chunks
                WHERE (course_name = ? OR course_code = ?)
            """
            total_chunks = conn.execute(total_chunks_query, params).fetchone()[0]
        else:
            total_chunks_query = "SELECT COUNT(*) FROM chunks"
            total_chunks = conn.execute(total_chunks_query).fetchone()[0]

        # Count chunks with embeddings (chunks.vector is populated)
        if course_filter:
            chunks_embedded_query = """
                SELECT COUNT(*) FROM chunks
                WHERE (course_name = ? OR course_code = ?)
                AND vector IS NOT NULL AND vector != ''
            """
            chunks_embedded = conn.execute(chunks_embedded_query, params).fetchone()[0]
        else:
            chunks_embedded_query = "SELECT COUNT(*) FROM chunks WHERE vector IS NOT NULL AND vector != ''"
            chunks_embedded = conn.execute(chunks_embedded_query).fetchone()[0]

        chunks_pending = total_chunks - chunks_embedded

        # === FILES WITH CONTENT ===
        # Count files that have at least one chunk (i.e., successfully converted)
        if course_filter:
            files_with_content_query = """
                SELECT COUNT(DISTINCT f.uuid)
                FROM file f
                INNER JOIN chunks c ON f.uuid = c.file_uuid
                WHERE (f.course_name = ? OR f.course_code = ?)
            """
            files_with_content = conn.execute(files_with_content_query, params).fetchone()[0]
        else:
            files_with_content_query = """
                SELECT COUNT(DISTINCT f.uuid)
                FROM file f
                INNER JOIN chunks c ON f.uuid = c.file_uuid
            """
            files_with_content = conn.execute(files_with_content_query).fetchone()[0]

        return {
            # File-level statistics
            "total_files": total_files,
            "files_embedded": files_embedded,
            "files_pending": files_pending,
            "files_with_content": files_with_content,
            "files_completion_percentage": (files_embedded / total_files * 100) if total_files > 0 else 0,

            # Chunk-level statistics
            "total_chunks": total_chunks,
            "chunks_embedded": chunks_embedded,
            "chunks_pending": chunks_pending,
            "chunks_completion_percentage": (chunks_embedded / total_chunks * 100) if total_chunks > 0 else 0,

            # Backward compatibility
            "total_embedded": files_embedded,  # For backward compatibility
            "embedded_files": files_embedded,  # Alias for clarity
            "completion_percentage": (files_embedded / total_files * 100) if total_files > 0 else 0
        }

    finally:
        conn.close()


if __name__ == "__main__":
    COLLECTIVE_DB_PATH = "/home/bot/bot/yk/YK_final/courses_out/collective_metadata.db"
    COURSES_OUT_DIR = "/home/bot/bot/yk/YK_final/courses_out/ROAR Academy"

    # Check current status
    status = check_embedding_status(COLLECTIVE_DB_PATH, course_filter=None)
    print("Current embedding status:", status)
    
    # Run embedding process
    results = embed_files_from_markdown(
        db_path=COLLECTIVE_DB_PATH,
        data_dir=COURSES_OUT_DIR,
        course_filter=None,  # Set to None to process all courses
        force_recompute=False  # Set to True to recompute existing embeddings
    )

    
    print("\nEmbedding results:")
    print(f"Processed: {results['processed']}")
    print(f"Errors: {results['errors']}")
    print(f"Status: {results['status']}")