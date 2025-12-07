"""SQLAlchemy models for database tables."""

from datetime import datetime
from sqlalchemy import Column, Integer, String, Text, DateTime, Boolean, JSON
from sqlalchemy.sql import func
from .connection import Base


class DocumentModel(Base):
    """Document table model."""
    
    __tablename__ = "documents"
    
    id = Column(Integer, primary_key=True, index=True)
    file_path = Column(String, unique=True, index=True, nullable=False)
    content = Column(Text, nullable=False)
    last_modified = Column(DateTime, nullable=False)
    file_hash = Column(String, nullable=False, index=True)
    module_id = Column(String, nullable=True)
    embedding_status = Column(String, default="pending", nullable=False)
    created_at = Column(DateTime, server_default=func.now(), nullable=False)
    updated_at = Column(DateTime, server_default=func.now(), onupdate=func.now(), nullable=False)


class ChunkModel(Base):
    """Chunk table model."""
    
    __tablename__ = "chunks"
    
    id = Column(Integer, primary_key=True, index=True)
    document_id = Column(Integer, nullable=False, index=True)
    chunk_index = Column(Integer, nullable=False)
    content = Column(Text, nullable=False)
    embedding_id = Column(String, nullable=True, index=True)  # Qdrant point ID
    char_count = Column(Integer, nullable=False)
    has_code = Column(Boolean, default=False, nullable=False)
    headers = Column(JSON, default=list, nullable=False)  # List of [level, text]
    created_at = Column(DateTime, server_default=func.now(), nullable=False)

