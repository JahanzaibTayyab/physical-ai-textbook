"""Database connection and initialization."""

import os
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession, async_sessionmaker
from sqlalchemy.orm import declarative_base

# Database URL from environment
DATABASE_URL = os.getenv("NEON_DATABASE_URL")
if not DATABASE_URL:
    raise ValueError("NEON_DATABASE_URL environment variable is required")

# Ensure asyncpg driver and handle SSL
if not DATABASE_URL.startswith("postgresql+asyncpg://"):
    if DATABASE_URL.startswith("postgresql://"):
        DATABASE_URL = DATABASE_URL.replace("postgresql://", "postgresql+asyncpg://", 1)
    
    # Remove sslmode from URL if present (asyncpg uses different SSL handling)
    if "?sslmode" in DATABASE_URL:
        # Extract base URL and params
        base_url, params = DATABASE_URL.split("?", 1)
        # Remove sslmode from params
        param_dict = {}
        for param in params.split("&"):
            if "=" in param:
                key, value = param.split("=", 1)
                if key != "sslmode":
                    param_dict[key] = value
        # Reconstruct URL
        if param_dict:
            DATABASE_URL = base_url + "?" + "&".join(f"{k}={v}" for k, v in param_dict.items())
        else:
            DATABASE_URL = base_url

# Create engine
engine = create_async_engine(
    DATABASE_URL,
    echo=False,  # Set to True for SQL logging
    future=True,
)

# Session factory
AsyncSessionLocal = async_sessionmaker(
    engine,
    class_=AsyncSession,
    expire_on_commit=False,
)

Base = declarative_base()


async def get_db_engine():
    """Get database engine."""
    return engine


async def init_db():
    """Initialize database tables."""
    from .models import DocumentModel, ChunkModel  # Import models to register them
    
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)


async def get_session() -> AsyncSession:
    """Get database session."""
    async with AsyncSessionLocal() as session:
        try:
            yield session
        finally:
            await session.close()

