#!/usr/bin/env python3
"""Test URDF search functionality."""

import asyncio
import sys
from pathlib import Path
from dotenv import load_dotenv

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from rag_chatbot_backend.services.embedding_service import get_embedding_service
from rag_chatbot_backend.services.vector_service import get_vector_service
from rag_chatbot_backend.database.repositories.chunk_repo import ChunkRepository

# Load environment variables
load_dotenv(Path(__file__).parent.parent / ".env")

async def test_search():
    """Test searching for URDF content."""
    print("Testing URDF search...")
    query = "What is URDF?"
    
    try:
        # Generate query embedding
        embedding_service = get_embedding_service()
        query_embedding = await embedding_service.generate_embedding(
            query,
            task_type="RETRIEVAL_QUERY",
            output_dimensionality=768
        )
        print(f"✓ Generated embedding (length: {len(query_embedding)})")
        
        # Search in Qdrant
        vector_service = get_vector_service()
        results = await vector_service.search_vectors(query_embedding, limit=5)
        print(f"✓ Found {len(results)} results from Qdrant")
        
        if not results:
            print("❌ No results from Qdrant!")
            return
        
        # Retrieve chunk contents
        chunk_repo = ChunkRepository()
        context_parts = []
        
        for i, result in enumerate(results):
            print(f"\nResult {i+1}:")
            print(f"  ID: {result.get('id')}")
            print(f"  Score: {result.get('score', 'N/A')}")
            payload = result.get("payload", {})
            print(f"  Payload: {payload}")
            
            chunk_id = payload.get("chunk_id")
            if chunk_id:
                chunk = await chunk_repo.get_by_id(chunk_id)
                if chunk:
                    print(f"  ✓ Retrieved chunk {chunk_id}")
                    context_parts.append(chunk.content[:200] + "...")
                else:
                    print(f"  ✗ Chunk {chunk_id} not found in database")
            elif result.get("id"):
                embedding_id = str(result["id"])
                chunks = await chunk_repo.get_by_embedding_ids([embedding_id])
                if chunks:
                    print(f"  ✓ Retrieved chunk by embedding_id {embedding_id}")
                    context_parts.append(chunks[0].content[:200] + "...")
                else:
                    print(f"  ✗ No chunk found for embedding_id {embedding_id}")
        
        print("\n" + "="*60)
        print("SEARCH RESULT:")
        print("="*60)
        if context_parts:
            print(f"Found {len(context_parts)} relevant chunks")
            for i, part in enumerate(context_parts, 1):
                print(f"\nChunk {i}:\n{part}")
        else:
            print("❌ No chunks retrieved!")
        print("="*60)
        
    except Exception as e:
        import traceback
        print(f"❌ Error: {e}")
        print(traceback.format_exc())

if __name__ == "__main__":
    asyncio.run(test_search())

