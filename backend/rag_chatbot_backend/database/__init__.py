"""Database module for RAG chatbot backend."""

from .connection import get_db_engine, init_db

__all__ = ["get_db_engine", "init_db"]

