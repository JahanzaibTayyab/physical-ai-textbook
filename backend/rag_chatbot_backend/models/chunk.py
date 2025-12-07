"""Chunk model for storing document chunks."""

from datetime import datetime
from typing import Optional, List
from pydantic import BaseModel


class Chunk(BaseModel):
    """Chunk model representing a piece of a document."""
    
    id: Optional[int] = None
    document_id: int
    chunk_index: int
    content: str
    embedding_id: Optional[str] = None  # Qdrant point ID
    char_count: int
    has_code: bool = False
    headers: List[tuple] = []  # List of (level, text) tuples
    created_at: Optional[datetime] = None
    
    class Config:
        from_attributes = True
        populate_by_name = True

