"""
RAG tools for OpenAI Agents SDK.

These tools allow the agent to search the textbook and answer questions.
"""

from typing import Annotated
from agents import function_tool
from ..services.embedding_service import embedding_service
from ..services.vector_service import get_vector_service
from ..database.repositories.chunk_repo import ChunkRepository


@function_tool
async def search_textbook(
    query: Annotated[str, "The user's question about the textbook"]
) -> str:
    """
    Search the textbook content for relevant information.
    
    This tool searches the vector database and returns relevant context
    from the textbook to help answer the user's question.
    
    Args:
        query: The question to search for
        
    Returns:
        Relevant context from the textbook
    """
    try:
        # Generate query embedding
        query_embedding = await embedding_service.generate_embedding(query)
        
        # Search in Qdrant
        vector_service = get_vector_service()
        results = await vector_service.search_vectors(query_embedding, limit=5)
        
        if not results:
            return "No relevant information found in the textbook."
        
        # Retrieve chunk contents from database
        chunk_repo = ChunkRepository()
        context_parts = []
        
        for result in results:
            # Qdrant returns payload with chunk_id or we use the point ID
            payload = result.get("payload", {})
            chunk_id = payload.get("chunk_id")
            if not chunk_id and result.get("id"):
                # Try to get chunk by embedding_id (Qdrant point ID)
                embedding_id = str(result["id"])
                chunks = await chunk_repo.get_by_embedding_ids([embedding_id])
                if chunks:
                    context_parts.append(chunks[0].content)
            elif chunk_id:
                chunk = await chunk_repo.get_by_id(chunk_id)
                if chunk:
                    context_parts.append(chunk.content)
        
        if not context_parts:
            return "No relevant information found in the textbook."
        
        context = "\n\n---\n\n".join(context_parts)
        return f"Relevant context from the textbook:\n\n{context}"
    
    except Exception as e:
        return f"Error searching textbook: {str(e)}"


@function_tool
async def answer_from_selected_text(
    selected_text: Annotated[str, "The text selected by the user"],
    question: Annotated[str, "The question about the selected text"]
) -> str:
    """
    Answer questions based only on the selected text.
    
    This tool is used when the user has selected specific text and asks
    a question about it. It does not search the entire textbook.
    
    Args:
        selected_text: The text the user selected
        question: The question about the selected text
        
    Returns:
        Answer based on the selected text only
    """
    return f"Based on the selected text:\n\n{selected_text}\n\nQuestion: {question}\n\nPlease answer based only on the information provided in the selected text above."

