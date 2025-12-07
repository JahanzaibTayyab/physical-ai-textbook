"""
FastAPI main application.

Provides endpoints for the RAG chatbot.
"""

import os
import os
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from agents import Agent, Runner, RunConfig
from agents.extensions.memory import SQLAlchemySession
from sqlalchemy.ext.asyncio import create_async_engine
from dotenv import load_dotenv

from ..services.gemini_model_provider import get_gemini_provider
from ..services.rag_tools import search_textbook, answer_from_selected_text

# Load environment variables
load_dotenv()

# Initialize FastAPI app
app = FastAPI(
    title="Physical AI Textbook RAG Chatbot API",
    description="RAG chatbot API for Physical AI & Humanoid Robotics textbook",
    version="1.0.0",
)

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify actual origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Database URL for SQLAlchemy Sessions
DATABASE_URL = os.getenv("NEON_DATABASE_URL")
if not DATABASE_URL:
    raise ValueError("NEON_DATABASE_URL environment variable is required")

# Ensure asyncpg driver
if not DATABASE_URL.startswith("postgresql+asyncpg://"):
    if DATABASE_URL.startswith("postgresql://"):
        DATABASE_URL = DATABASE_URL.replace("postgresql://", "postgresql+asyncpg://", 1)

# Create engine for SQLAlchemy Sessions
session_engine = create_async_engine(DATABASE_URL, echo=False)

# Create agent with RAG tools
agent = Agent(
    name="TextbookAssistant",
    instructions="""
    You are a helpful assistant for the Physical AI & Humanoid Robotics textbook.
    Answer questions based ONLY on the provided context from the textbook.
    
    When answering:
    1. Use the search_textbook tool to find relevant information
    2. Base your answer only on the retrieved context
    3. If the context doesn't contain enough information, say so
    4. Be concise, accurate, and cite relevant sections when possible
    5. For selected text questions, use answer_from_selected_text tool
    """,
    tools=[search_textbook, answer_from_selected_text],
)


# Request/Response models
class QueryRequest(BaseModel):
    """Query request model."""
    query: str
    user_id: str
    selected_text: str = None
    session_id: str = None


class QueryResponse(BaseModel):
    """Query response model."""
    answer: str
    session_id: str
    sources: list = None
    response_time_ms: int = None


@app.get("/")
async def root():
    """Root endpoint."""
    return {"message": "Physical AI Textbook RAG Chatbot API"}


@app.get("/health")
async def health():
    """Health check endpoint."""
    return {"status": "healthy"}


@app.post("/api/chat/query", response_model=QueryResponse)
async def query_chatbot(request: QueryRequest):
    """
    Query the chatbot.
    
    Args:
        request: Query request with user query and session info
        
    Returns:
        Query response with answer and session ID
    """
    import time
    start_time = time.time()
    
    try:
        # Create or get session
        session = SQLAlchemySession(
            user_id=request.user_id,
            engine=session_engine,
            create_tables=True,
        )
        
        # Build query
        if request.selected_text:
            user_query = (
                f"Based on this text: {request.selected_text}\n\n"
                f"Question: {request.query}"
            )
        else:
            user_query = request.query
        
        # Run agent with Gemini provider
        gemini_provider = get_gemini_provider()
        result = await Runner.run(
            agent,
            user_query,
            session=session,
            run_config=RunConfig(model_provider=gemini_provider),
        )
        
        response_time_ms = int((time.time() - start_time) * 1000)
        
        # Extract sources from tool calls if available
        sources = []
        if hasattr(result, "tool_calls"):
            for tool_call in result.tool_calls:
                if tool_call.name == "search_textbook":
                    sources.append({"type": "textbook_search", "tool": "search_textbook"})
        
        return QueryResponse(
            answer=result.final_output,
            session_id=session.session_id,
            sources=sources,
            response_time_ms=response_time_ms,
        )
    
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)

