"""SQLAlchemy models for database tables."""

from datetime import datetime
from sqlalchemy import Column, Integer, String, Text, DateTime, Boolean, JSON, ForeignKey
from sqlalchemy.orm import relationship
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


class UserModel(Base):
    """User table model for authentication."""

    __tablename__ = "users"

    id = Column(Integer, primary_key=True, index=True)
    email = Column(String, unique=True, index=True, nullable=False)
    password_hash = Column(String, nullable=False)
    created_at = Column(DateTime, server_default=func.now(), nullable=False)
    updated_at = Column(DateTime, server_default=func.now(), onupdate=func.now(), nullable=False)


class UserProfileModel(Base):
    """User profile table model for background information."""

    __tablename__ = "user_profiles"

    id = Column(Integer, primary_key=True, index=True)
    user_id = Column(Integer, ForeignKey("users.id"), unique=True, nullable=False, index=True)
    software_background = Column(Text, nullable=True)
    hardware_background = Column(Text, nullable=True)
    created_at = Column(DateTime, server_default=func.now(), nullable=False)
    updated_at = Column(DateTime, server_default=func.now(), onupdate=func.now(), nullable=False)


class PersonalizedContentModel(Base):
    """Personalized content cache table."""

    __tablename__ = "personalized_content"

    id = Column(Integer, primary_key=True, index=True)
    user_id = Column(Integer, ForeignKey("users.id"), nullable=False, index=True)
    chapter_path = Column(String, nullable=False, index=True)
    original_content = Column(Text, nullable=False)
    personalized_content = Column(Text, nullable=False)
    background_type = Column(String, nullable=False)  # "software" or "hardware"
    content_hash = Column(String, unique=True, nullable=False, index=True)
    created_at = Column(DateTime, server_default=func.now(), nullable=False)
    updated_at = Column(DateTime, server_default=func.now(), onupdate=func.now(), nullable=False)


class TranslationModel(Base):
    """Translation cache table."""

    __tablename__ = "translations"

    id = Column(Integer, primary_key=True, index=True)
    chapter_path = Column(String, nullable=False, index=True)
    language = Column(String, nullable=False)  # "ur" for Urdu
    original_content = Column(Text, nullable=False)
    translated_content = Column(Text, nullable=False)
    content_hash = Column(String, unique=True, nullable=False, index=True)
    created_at = Column(DateTime, server_default=func.now(), nullable=False)
    updated_at = Column(DateTime, server_default=func.now(), onupdate=func.now(), nullable=False)

