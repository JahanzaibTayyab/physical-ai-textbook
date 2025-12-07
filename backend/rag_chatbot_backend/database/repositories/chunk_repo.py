"""Repository for chunk database operations."""

from typing import Optional, List
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select
from ..models import ChunkModel
from ...models.chunk import Chunk


class ChunkRepository:
    """Repository for chunk operations."""
    
    def __init__(self, session: Optional[AsyncSession] = None):
        """
        Initialize repository.
        
        Args:
            session: Optional database session (will create one if not provided)
        """
        self.session = session
    
    async def get_by_id(self, chunk_id: int) -> Optional[Chunk]:
        """
        Get chunk by ID.
        
        Args:
            chunk_id: Chunk ID
            
        Returns:
            Chunk if found, None otherwise
        """
        if not self.session:
            from ..connection import _get_session_local
            AsyncSessionLocal = _get_session_local()
            async with AsyncSessionLocal() as session:
                stmt = select(ChunkModel).where(ChunkModel.id == chunk_id)
                result = await session.execute(stmt)
                model = result.scalar_one_or_none()
                if model:
                    return Chunk.model_validate(model)
                return None
        
        stmt = select(ChunkModel).where(ChunkModel.id == chunk_id)
        result = await self.session.execute(stmt)
        model = result.scalar_one_or_none()
        if model:
            return Chunk.model_validate(model)
        return None
    
    async def get_by_embedding_ids(
        self, embedding_ids: List[str]
    ) -> List[Chunk]:
        """
        Get chunks by embedding IDs.
        
        Args:
            embedding_ids: List of Qdrant embedding IDs
            
        Returns:
            List of chunks
        """
        if not self.session:
            from ..connection import _get_session_local
            AsyncSessionLocal = _get_session_local()
            async with AsyncSessionLocal() as session:
                stmt = select(ChunkModel).where(
                    ChunkModel.embedding_id.in_(embedding_ids)
                )
                result = await session.execute(stmt)
                models = result.scalars().all()
                return [Chunk.model_validate(model) for model in models]
        
        stmt = select(ChunkModel).where(
            ChunkModel.embedding_id.in_(embedding_ids)
        )
        result = await self.session.execute(stmt)
        models = result.scalars().all()
        return [Chunk.model_validate(model) for model in models]

