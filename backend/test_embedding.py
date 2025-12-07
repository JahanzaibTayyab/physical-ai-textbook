#!/usr/bin/env python3
"""Quick test for Gemini embedding generation."""

import asyncio
import sys
from pathlib import Path
from dotenv import load_dotenv

sys.path.insert(0, str(Path(__file__).parent))

load_dotenv()

from rag_chatbot_backend.services.embedding_service import get_embedding_service


async def test_embedding():
    """Test embedding generation."""
    print("🧪 Testing Gemini embedding generation...")
    try:
        service = get_embedding_service()
        text = "This is a test document about ROS 2 robotics framework."
        
        print("  Generating embedding...")
        embedding = await service.generate_embedding(text)
        
        print(f"  ✅ Embedding generated successfully!")
        print(f"     Dimensions: {len(embedding)}")
        print(f"     First 5 values: {embedding[:5]}")
        return True
    except Exception as e:
        print(f"  ❌ Error: {e}")
        return False


if __name__ == "__main__":
    success = asyncio.run(test_embedding())
    sys.exit(0 if success else 1)

