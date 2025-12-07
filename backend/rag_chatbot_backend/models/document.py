"""Document model for storing markdown file metadata."""

from datetime import datetime
from typing import Optional
from pydantic import BaseModel


class Document(BaseModel):
    """Document model representing a markdown file."""
    
    id: Optional[int] = None
    file_path: str
    content: str
    last_modified: datetime
    file_hash: str
    module_id: Optional[str] = None
    embedding_status: str = "pending"  # pending, complete, needs_update
    created_at: Optional[datetime] = None
    updated_at: Optional[datetime] = None
    
    class Config:
        from_attributes = True
        populate_by_name = True

