"""
Embedding service using Gemini API.

Generates embeddings for text chunks using Gemini's embedding model.
"""

import os
from typing import List
import google.generativeai as genai

# Configure Gemini
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
if not GEMINI_API_KEY:
    raise ValueError("GEMINI_API_KEY environment variable is required")

genai.configure(api_key=GEMINI_API_KEY)


class EmbeddingService:
    """Service for generating embeddings using Gemini API."""
    
    def __init__(self, model_name: str = "models/embedding-001"):
        """
        Initialize embedding service.
        
        Args:
            model_name: Gemini embedding model name
        """
        self.model_name = model_name
    
    async def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for a single text.
        
        Args:
            text: Text to embed
            
        Returns:
            List of floats representing the embedding vector
        """
        try:
            # Gemini embedding API
            result = genai.embed_content(
                model=self.model_name,
                content=text,
                task_type="retrieval_document"
            )
            return result["embedding"]
        except Exception as e:
            raise RuntimeError(f"Failed to generate embedding: {e}")
    
    async def generate_embeddings_batch(
        self, texts: List[str]
    ) -> List[List[float]]:
        """
        Generate embeddings for multiple texts.
        
        Args:
            texts: List of texts to embed
            
        Returns:
            List of embedding vectors
        """
        embeddings = []
        for text in texts:
            embedding = await self.generate_embedding(text)
            embeddings.append(embedding)
        return embeddings


# Global service instance
embedding_service = EmbeddingService()

