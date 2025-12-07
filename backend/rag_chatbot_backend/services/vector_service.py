"""
Vector service for Qdrant operations.

Handles storing and searching embeddings in Qdrant Cloud.
"""

import os
from typing import List, Optional
from qdrant_client import AsyncQdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct

# Qdrant configuration (lazy loaded)
COLLECTION_NAME = "textbook_chunks"

def _get_qdrant_config():
    """Get Qdrant configuration from environment (lazy loading)."""
    url = os.getenv("QDRANT_URL")
    api_key = os.getenv("QDRANT_API_KEY")
    if not url or not api_key:
        raise ValueError(
            "QDRANT_URL and QDRANT_API_KEY environment variables are required"
        )
    return url, api_key


class VectorService:
    """Service for Qdrant vector operations."""
    
    def __init__(
        self,
        url: Optional[str] = None,
        api_key: Optional[str] = None,
        collection_name: str = COLLECTION_NAME,
    ):
        """
        Initialize vector service.
        
        Args:
            url: Qdrant URL (defaults to QDRANT_URL env var)
            api_key: Qdrant API key (defaults to QDRANT_API_KEY env var)
            collection_name: Collection name for vectors
        """
        self.collection_name = collection_name
        
        # Lazy load config if not provided
        if url and api_key:
            self.url = url
            self.api_key = api_key
        else:
            self.url, self.api_key = _get_qdrant_config()
        
        self.client = AsyncQdrantClient(
            url=self.url,
            api_key=self.api_key,
        )
    
    async def initialize_collection(self, vector_size: int = 768):
        """
        Initialize Qdrant collection if it doesn't exist.
        
        Args:
            vector_size: Size of embedding vectors (Gemini: 768)
        """
        try:
            collections = await self.client.get_collections()
            collection_names = [c.name for c in collections.collections]
            
            if self.collection_name not in collection_names:
                await self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(
                        size=vector_size,
                        distance=Distance.COSINE,
                    ),
                )
        except Exception as e:
            raise RuntimeError(f"Failed to initialize collection: {e}")
    
    async def upsert_vectors(
        self, points: List[PointStruct]
    ):
        """
        Upsert vectors into Qdrant.
        
        Args:
            points: List of PointStruct objects with id, vector, and payload
        """
        try:
            await self.client.upsert(
                collection_name=self.collection_name,
                points=points,
            )
        except Exception as e:
            raise RuntimeError(f"Failed to upsert vectors: {e}")
    
    async def search_vectors(
        self, query_vector: List[float], limit: int = 5
    ) -> List[dict]:
        """
        Search for similar vectors.
        
        Args:
            query_vector: Query embedding vector
            limit: Number of results to return
            
        Returns:
            List of search results with id, score, and payload
        """
        try:
            # Use query_points for async client - pass vector directly
            results = await self.client.query_points(
                collection_name=self.collection_name,
                query=query_vector,  # Pass vector directly
                limit=limit,
            )
            
            return [
                {
                    "id": str(point.id),  # Convert to string for consistency
                    "score": point.score,
                    "payload": point.payload or {},
                }
                for point in results.points
            ]
        except Exception as e:
            raise RuntimeError(f"Failed to search vectors: {e}")


# Global service instance (will be initialized with env vars)
vector_service: Optional[VectorService] = None

def get_vector_service() -> VectorService:
    """Get or create vector service instance."""
    global vector_service
    if vector_service is None:
        vector_service = VectorService()
    return vector_service

