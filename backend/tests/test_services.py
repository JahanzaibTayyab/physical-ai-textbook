"""
Unit tests for services according to the plan.

Tests embedding, chunking, and vector services.
"""

import pytest
from rag_chatbot_backend.services.chunking_service import chunking_service
from rag_chatbot_backend.services.embedding_service import embedding_service
from rag_chatbot_backend.services.vector_service import get_vector_service


@pytest.mark.asyncio
async def test_chunking_service():
    """Test chunking service respects markdown structure."""
    content = """# Introduction

This is a paragraph.

```python
def hello():
    print("world")
```

Another paragraph here.
"""
    chunks = chunking_service.chunk_document(content, "test.md")
    
    assert len(chunks) > 0
    assert all("content" in chunk for chunk in chunks)
    assert all("chunk_index" in chunk for chunk in chunks)
    
    # Check code block preservation
    code_chunks = [c for c in chunks if c.get("has_code")]
    if code_chunks:
        assert "```python" in code_chunks[0]["content"]


@pytest.mark.asyncio
async def test_embedding_service():
    """Test embedding generation."""
    text = "This is a test document about ROS 2."
    
    try:
        embedding = await embedding_service.generate_embedding(text)
        assert isinstance(embedding, list)
        assert len(embedding) == 768  # Gemini embedding dimension
        assert all(isinstance(x, float) for x in embedding)
    except Exception as e:
        # Skip if quota exceeded
        if "quota" in str(e).lower() or "429" in str(e):
            pytest.skip(f"Gemini API quota exceeded: {e}")
        raise


@pytest.mark.asyncio
async def test_vector_service_initialization():
    """Test Qdrant vector service initialization."""
    try:
        service = get_vector_service()
        await service.initialize_collection(vector_size=768)
        
        assert service.url is not None
        assert service.api_key is not None
        assert service.collection_name == "textbook_chunks"
    except Exception as e:
        if "QDRANT" in str(e) or "environment" in str(e).lower():
            pytest.skip(f"Qdrant not configured: {e}")
        raise

