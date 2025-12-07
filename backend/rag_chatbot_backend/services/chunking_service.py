"""
Intelligent chunking service for markdown documents.

Chunks content respecting markdown structure with overlap.
"""

from typing import List, Dict, Any
from ..utils.markdown_parser import split_markdown_by_structure


class ChunkingService:
    """Service for chunking markdown documents intelligently."""
    
    def __init__(
        self, max_chunk_size: int = 800, overlap: int = 150
    ):
        """
        Initialize chunking service.
        
        Args:
            max_chunk_size: Maximum characters per chunk (500-1000 recommended)
            overlap: Character overlap between chunks (100-200 recommended)
        """
        self.max_chunk_size = max_chunk_size
        self.overlap = overlap
    
    def chunk_document(
        self, content: str, file_path: str
    ) -> List[Dict[str, Any]]:
        """
        Chunk a markdown document.
        
        Args:
            content: Document content
            file_path: Path to the document file
            
        Returns:
            List of chunk dictionaries with metadata
        """
        chunks_data = split_markdown_by_structure(
            content, self.max_chunk_size, self.overlap
        )
        
        chunks = []
        for idx, (chunk_text, metadata) in enumerate(chunks_data):
            chunk = {
                "content": chunk_text,
                "chunk_index": idx,
                "file_path": file_path,
                "has_code": metadata.get("has_code", False),
                "headers": metadata.get("headers", []),
                "char_count": len(chunk_text),
            }
            chunks.append(chunk)
        
        return chunks


# Global service instance
chunking_service = ChunkingService(max_chunk_size=800, overlap=150)

