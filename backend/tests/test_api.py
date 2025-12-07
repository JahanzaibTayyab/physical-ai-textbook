"""
Integration tests for FastAPI endpoints.

Tests API endpoints according to the specification.
"""

import pytest
from fastapi.testclient import TestClient
from rag_chatbot_backend.api.main import app

client = TestClient(app)


def test_root_endpoint():
    """Test root endpoint."""
    response = client.get("/")
    assert response.status_code == 200
    assert "message" in response.json()


def test_health_endpoint():
    """Test health check endpoint."""
    response = client.get("/health")
    assert response.status_code == 200
    assert response.json() == {"status": "healthy"}


def test_query_endpoint_structure():
    """Test query endpoint accepts correct request structure."""
    response = client.post(
        "/api/chat/query",
        json={
            "query": "What is ROS 2?",
            "user_id": "test-user-123"
        }
    )
    
    # Should either succeed or fail with proper error (not 422 validation error)
    assert response.status_code != 422
    
    if response.status_code == 200:
        data = response.json()
        assert "answer" in data
        assert "session_id" in data
        assert isinstance(data["answer"], str)


def test_query_with_selected_text():
    """Test query endpoint with selected text."""
    response = client.post(
        "/api/chat/query",
        json={
            "query": "Explain this",
            "user_id": "test-user-123",
            "selected_text": "ROS 2 is a robotics framework"
        }
    )
    
    assert response.status_code != 422
    
    if response.status_code == 200:
        data = response.json()
        assert "answer" in data
        assert "session_id" in data

