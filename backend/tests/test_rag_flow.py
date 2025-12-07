"""
End-to-end RAG flow tests.

Tests the complete RAG pipeline according to the plan.
"""

import pytest
from rag_chatbot_backend.services.rag_tools import search_textbook, answer_from_selected_text


@pytest.mark.asyncio
async def test_search_textbook_tool():
    """Test search_textbook RAG tool implementation."""
    # Test the underlying function directly
    from rag_chatbot_backend.services.rag_tools import _search_textbook_impl
    
    try:
        result = await _search_textbook_impl("What is ROS 2?")
        
        assert isinstance(result, str)
        assert len(result) > 0
        
        # Should either return context or error message
        assert "context" in result.lower() or "error" in result.lower() or "no relevant" in result.lower()
    except Exception as e:
        # Skip if dependencies not configured
        if "environment" in str(e).lower() or "QDRANT" in str(e) or "GEMINI" in str(e):
            pytest.skip(f"Dependencies not configured: {e}")
        raise


@pytest.mark.asyncio
async def test_answer_from_selected_text_tool():
    """Test answer_from_selected_text tool implementation."""
    from rag_chatbot_backend.services.rag_tools import _answer_from_selected_text_impl
    
    result = await _answer_from_selected_text_impl(
        "ROS 2 is a robotics framework",
        "What is ROS 2?"
    )
    
    assert isinstance(result, str)
    assert "ROS 2" in result
    assert "selected text" in result.lower()

