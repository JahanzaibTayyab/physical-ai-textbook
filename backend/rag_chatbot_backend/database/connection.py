"""Database connection and initialization."""

import os
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession, async_sessionmaker
from sqlalchemy.orm import declarative_base

# Lazy database initialization
_engine = None
_AsyncSessionLocal = None

def _get_database_url():
    """Get and process database URL."""
    DATABASE_URL = os.getenv("NEON_DATABASE_URL")
    if not DATABASE_URL:
        raise ValueError("NEON_DATABASE_URL environment variable is required")
    
    # Ensure asyncpg driver and handle SSL
    if not DATABASE_URL.startswith("postgresql+asyncpg://"):
        if DATABASE_URL.startswith("postgresql://"):
            DATABASE_URL = DATABASE_URL.replace("postgresql://", "postgresql+asyncpg://", 1)
        
        # Remove unsupported parameters from URL (asyncpg doesn't support sslmode, channel_binding)
        if "?" in DATABASE_URL:
            # Extract base URL and params
            base_url, params = DATABASE_URL.split("?", 1)
            # Remove unsupported params
            param_dict = {}
            for param in params.split("&"):
                if "=" in param:
                    key, value = param.split("=", 1)
                    # Keep only supported parameters, remove sslmode and channel_binding
                    if key not in ["sslmode", "channel_binding"]:
                        param_dict[key] = value
            # Reconstruct URL
            if param_dict:
                DATABASE_URL = base_url + "?" + "&".join(f"{k}={v}" for k, v in param_dict.items())
            else:
                DATABASE_URL = base_url
    
    return DATABASE_URL

def _get_engine():
    """Get or create database engine (lazy initialization)."""
    global _engine
    if _engine is None:
        DATABASE_URL = _get_database_url()
        _engine = create_async_engine(
            DATABASE_URL,
            echo=False,  # Set to True for SQL logging
            future=True,
        )
    return _engine

def _get_session_local():
    """Get or create session factory (lazy initialization)."""
    global _AsyncSessionLocal
    if _AsyncSessionLocal is None:
        engine = _get_engine()
        _AsyncSessionLocal = async_sessionmaker(
            engine,
            class_=AsyncSession,
            expire_on_commit=False,
        )
    return _AsyncSessionLocal

# For backward compatibility
engine = property(lambda self: _get_engine())
AsyncSessionLocal = property(lambda self: _get_session_local())

Base = declarative_base()


async def get_db_engine():
    """Get database engine."""
    return _get_engine()


async def init_db():
    """Initialize database tables."""
    from .models import DocumentModel, ChunkModel  # Import models to register them
    
    engine = _get_engine()
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)


async def get_session() -> AsyncSession:
    """Get database session."""
    AsyncSessionLocal = _get_session_local()
    async with AsyncSessionLocal() as session:
        try:
            yield session
        finally:
            await session.close()

