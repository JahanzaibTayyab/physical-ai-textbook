"""
Embedding service using Gemini API.

Generates embeddings for text chunks using Gemini's embedding model.
Uses the official Gemini API client as per documentation:
https://ai.google.dev/gemini-api/docs/embeddings
"""

import os
from typing import List
from google import genai

# Lazy Gemini client initialization
_genai_client = None

def _get_genai_client():
    """Get or create Gemini client (lazy initialization)."""
    global _genai_client
    if _genai_client is None:
        GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
        if not GEMINI_API_KEY:
            raise ValueError("GEMINI_API_KEY environment variable is required")
        _genai_client = genai.Client(api_key=GEMINI_API_KEY)
    return _genai_client


class EmbeddingService:
    """Service for generating embeddings using Gemini API."""
    
    def __init__(self, model_name: str = "gemini-embedding-001"):
        """
        Initialize embedding service.
        
        Args:
            model_name: Gemini embedding model name (default: gemini-embedding-001)
        """
        self.model_name = model_name
    
    async def generate_embedding(
        self, 
        text: str, 
        task_type: str = "RETRIEVAL_DOCUMENT",
        output_dimensionality: int = 768
    ) -> List[float]:
        """
        Generate embedding for a single text.
        
        Uses the official Gemini API client as per:
        https://ai.google.dev/gemini-api/docs/embeddings
        
        Args:
            text: Text to embed
            task_type: Task type for optimization (RETRIEVAL_DOCUMENT, RETRIEVAL_QUERY, etc.)
            output_dimensionality: Embedding dimension (768, 1536, or 3072)
            
        Returns:
            List of floats representing the embedding vector
        """
        client = _get_genai_client()
        try:
            # Use official Gemini API client
            # Reference: https://ai.google.dev/gemini-api/docs/embeddings
            from google.genai import types
            
            result = client.models.embed_content(
                model=self.model_name,
                contents=text,
                config=types.EmbedContentConfig(
                    taskType=task_type,
                    outputDimensionality=output_dimensionality
                )
            )
            # Extract embedding values
            if result.embeddings and len(result.embeddings) > 0:
                return result.embeddings[0].values
            else:
                raise RuntimeError("No embeddings returned from API")
        except Exception as e:
            raise RuntimeError(f"Failed to generate embedding: {e}")
    
    async def generate_embeddings_batch(
        self, 
        texts: List[str],
        task_type: str = "RETRIEVAL_DOCUMENT",
        output_dimensionality: int = 768
    ) -> List[List[float]]:
        """
        Generate embeddings for multiple texts in a single API call.
        
        This is more efficient than calling generate_embedding multiple times
        and reduces API quota usage.
        
        Args:
            texts: List of texts to embed
            task_type: Task type for optimization
            output_dimensionality: Embedding dimension
            
        Returns:
            List of embedding vectors
        """
        client = _get_genai_client()
        try:
            # Batch embedding generation - more efficient
            # Reference: https://ai.google.dev/gemini-api/docs/embeddings
            from google.genai import types
            
            result = client.models.embed_content(
                model=self.model_name,
                contents=texts,  # Pass list directly for batch processing
                config=types.EmbedContentConfig(
                    taskType=task_type,
                    outputDimensionality=output_dimensionality
                )
            )
            # Extract all embeddings
            return [emb.values for emb in result.embeddings]
        except Exception as e:
            raise RuntimeError(f"Failed to generate batch embeddings: {e}")


# Global service instance (lazy initialization)
_embedding_service = None

def get_embedding_service():
    """Get or create global embedding service."""
    global _embedding_service
    if _embedding_service is None:
        _embedding_service = EmbeddingService()
    return _embedding_service

# For backward compatibility - create instance
embedding_service = EmbeddingService()

