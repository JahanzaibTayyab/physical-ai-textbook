#!/usr/bin/env python3
"""
Document processing script.

Processes all markdown files from the docs directory, chunks them,
generates embeddings, and stores them in Qdrant and Postgres.
"""

import asyncio
import os
import sys
import hashlib
from pathlib import Path
from datetime import datetime
from typing import List
from dotenv import load_dotenv

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from sqlalchemy.ext.asyncio import AsyncSession
from qdrant_client.models import PointStruct

from rag_chatbot_backend.database.connection import AsyncSessionLocal, init_db
from rag_chatbot_backend.database.models import DocumentModel, ChunkModel
from rag_chatbot_backend.services.chunking_service import chunking_service
from rag_chatbot_backend.services.embedding_service import embedding_service
from rag_chatbot_backend.services.vector_service import get_vector_service
from sqlalchemy import select

# Load environment variables
load_dotenv()

# Path to docs directory (relative to project root)
DOCS_DIR = Path(__file__).parent.parent.parent / "docs"


async def get_file_hash(file_path: Path) -> str:
    """Calculate SHA256 hash of file."""
    with open(file_path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


async def process_document(
    file_path: Path, session: AsyncSession
) -> tuple[DocumentModel, List[ChunkModel]]:
    """
    Process a single markdown document.
    
    Returns:
        Tuple of (DocumentModel, List[ChunkModel])
    """
    # Read file
    with open(file_path, "r", encoding="utf-8") as f:
        content = f.read()
    
    # Get file metadata
    file_hash = await get_file_hash(file_path)
    last_modified = datetime.fromtimestamp(file_path.stat().st_mtime)
    relative_path = str(file_path.relative_to(DOCS_DIR.parent))
    
    # Check if document exists
    stmt = select(DocumentModel).where(
        DocumentModel.file_path == relative_path
    )
    result = await session.execute(stmt)
    existing_doc = result.scalar_one_or_none()
    
    # Check if needs update
    if existing_doc and existing_doc.file_hash == file_hash:
        print(f"  ⏭️  Skipping {relative_path} (no changes)")
        return existing_doc, []
    
    # Extract module ID from path
    module_id = None
    if "module-1" in relative_path:
        module_id = "module-1-ros2"
    elif "module-2" in relative_path:
        module_id = "module-2-simulation"
    elif "module-3" in relative_path:
        module_id = "module-3-isaac"
    elif "module-4" in relative_path:
        module_id = "module-4-vla"
    
    # Create or update document
    if existing_doc:
        existing_doc.content = content
        existing_doc.file_hash = file_hash
        existing_doc.last_modified = last_modified
        existing_doc.embedding_status = "needs_update"
        document = existing_doc
    else:
        document = DocumentModel(
            file_path=relative_path,
            content=content,
            file_hash=file_hash,
            last_modified=last_modified,
            module_id=module_id,
            embedding_status="pending",
        )
        session.add(document)
    
    await session.flush()
    await session.refresh(document)
    
    # Delete old chunks if updating
    if existing_doc:
        stmt = select(ChunkModel).where(ChunkModel.document_id == document.id)
        result = await session.execute(stmt)
        old_chunks = result.scalars().all()
        for chunk in old_chunks:
            await session.delete(chunk)
    
    # Chunk document
    chunks_data = chunking_service.chunk_document(content, relative_path)
    
    # Create chunk models
    chunks = []
    for chunk_data in chunks_data:
        chunk = ChunkModel(
            document_id=document.id,
            chunk_index=chunk_data["chunk_index"],
            content=chunk_data["content"],
            char_count=chunk_data["char_count"],
            has_code=chunk_data["has_code"],
            headers=chunk_data["headers"],
        )
        session.add(chunk)
        chunks.append(chunk)
    
    await session.flush()
    
    # Generate embeddings and store in Qdrant
    vector_service = get_vector_service()
    await vector_service.initialize_collection(vector_size=768)
    
    points = []
    for chunk in chunks:
        await session.refresh(chunk)
        
        # Generate embedding
        embedding = await embedding_service.generate_embedding(chunk.content)
        
        # Create Qdrant point
        point_id = f"{document.id}_{chunk.id}"
        point = PointStruct(
            id=point_id,
            vector=embedding,
            payload={
                "chunk_id": chunk.id,
                "document_id": document.id,
                "file_path": relative_path,
                "chunk_index": chunk.chunk_index,
                "has_code": chunk.has_code,
            },
        )
        points.append(point)
        
        # Update chunk with embedding ID
        chunk.embedding_id = point_id
    
    # Upsert to Qdrant
    if points:
        await vector_service.upsert_vectors(points)
    
    # Update document status
    document.embedding_status = "complete"
    
    await session.commit()
    
    return document, chunks


async def main():
    """Main processing function."""
    print("🚀 Starting document processing...")
    print(f"📁 Docs directory: {DOCS_DIR}")
    
    if not DOCS_DIR.exists():
        print(f"❌ Error: Docs directory not found at {DOCS_DIR}")
        return
    
    # Initialize database
    print("\n📊 Initializing database...")
    await init_db()
    print("✅ Database initialized")
    
    # Find all markdown files
    markdown_files = list(DOCS_DIR.rglob("*.md"))
    print(f"\n📄 Found {len(markdown_files)} markdown files")
    
    # Process documents
    async with AsyncSessionLocal() as session:
        for i, file_path in enumerate(markdown_files, 1):
            print(f"\n[{i}/{len(markdown_files)}] Processing {file_path.name}...")
            try:
                doc, chunks = await process_document(file_path, session)
                print(f"  ✅ Processed: {len(chunks)} chunks created")
            except Exception as e:
                print(f"  ❌ Error processing {file_path.name}: {e}")
                await session.rollback()
                continue
    
    print("\n✅ Document processing complete!")


if __name__ == "__main__":
    asyncio.run(main())

