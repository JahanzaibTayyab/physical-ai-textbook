#!/usr/bin/env python3
"""Check if URDF documents are in the database."""

import asyncio
import sys
from pathlib import Path
from dotenv import load_dotenv

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from sqlalchemy import select, func
from rag_chatbot_backend.database.connection import _get_session_local
from rag_chatbot_backend.database.models import DocumentModel, ChunkModel

# Load environment variables
load_dotenv(Path(__file__).parent.parent / ".env")

async def check_urdf():
    """Check for URDF documents and chunks."""
    AsyncSessionLocal = _get_session_local()
    async with AsyncSessionLocal() as session:
        # Check for URDF documents
        stmt = select(DocumentModel).where(DocumentModel.file_path.like('%urdf%'))
        result = await session.execute(stmt)
        docs = result.scalars().all()
        print(f'Found {len(docs)} URDF-related documents:')
        for doc in docs:
            print(f'  - {doc.file_path} (status: {doc.embedding_status})')
        
        # Check chunks for URDF documents
        if docs:
            doc_ids = [doc.id for doc in docs]
            stmt = select(func.count(ChunkModel.id)).where(ChunkModel.document_id.in_(doc_ids))
            result = await session.execute(stmt)
            chunk_count = result.scalar()
            print(f'\nTotal chunks for URDF documents: {chunk_count}')
        
        # Check total chunks
        stmt = select(func.count(ChunkModel.id))
        result = await session.execute(stmt)
        total_chunks = result.scalar()
        print(f'\nTotal chunks in database: {total_chunks}')

if __name__ == "__main__":
    asyncio.run(check_urdf())

