#!/usr/bin/env python3
"""
Quick connection test script.

Tests connections to Qdrant, Postgres, and Gemini API.
"""

import asyncio
import os
import sys
from pathlib import Path
from dotenv import load_dotenv

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent))

load_dotenv()


async def test_qdrant():
    """Test Qdrant connection."""
    print("🔍 Testing Qdrant connection...")
    try:
        from rag_chatbot_backend.services.vector_service import get_vector_service
        
        service = get_vector_service()
        await service.initialize_collection(vector_size=768)
        print("  ✅ Qdrant connection successful!")
        print(f"     URL: {service.url}")
        print(f"     Collection: {service.collection_name}")
        return True
    except Exception as e:
        print(f"  ❌ Qdrant connection failed: {e}")
        return False


async def test_postgres():
    """Test Postgres connection."""
    print("\n🔍 Testing Postgres connection...")
    try:
        from rag_chatbot_backend.database.connection import _get_engine
        
        engine = _get_engine()
        from sqlalchemy import text
        async with engine.begin() as conn:
            await conn.execute(text("SELECT 1"))
        print("  ✅ Postgres connection successful!")
        return True
    except Exception as e:
        print(f"  ❌ Postgres connection failed: {e}")
        return False


async def test_gemini():
    """Test Gemini API."""
    print("\n🔍 Testing Gemini API...")
    try:
        from rag_chatbot_backend.services.embedding_service import embedding_service
        
        # Test embedding generation
        test_text = "This is a test"
        embedding = await embedding_service.generate_embedding(test_text)
        print(f"  ✅ Gemini API connection successful!")
        print(f"     Embedding dimension: {len(embedding)}")
        return True
    except Exception as e:
        print(f"  ❌ Gemini API connection failed: {e}")
        return False


async def main():
    """Run all tests."""
    print("🧪 Testing connections...\n")
    
    results = await asyncio.gather(
        test_qdrant(),
        test_postgres(),
        test_gemini(),
    )
    
    print("\n" + "=" * 50)
    if all(results):
        print("✅ All connections successful!")
        return 0
    else:
        print("❌ Some connections failed. Check your .env file.")
        return 1


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))

