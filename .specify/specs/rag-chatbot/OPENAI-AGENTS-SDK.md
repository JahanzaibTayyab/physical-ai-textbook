# OpenAI Agents SDK Integration Guide with Gemini API

This document outlines the integration of OpenAI Agents SDK for the RAG chatbot, using **Gemini API as a custom model provider** instead of OpenAI's API. This allows us to use Gemini for both embeddings and chat completions while leveraging the Agents SDK's powerful features.

## Why OpenAI Agents SDK?

The [OpenAI Agents SDK](https://openai.github.io/openai-agents-python/) provides:

- **Built-in agent loop**: Handles tool calling, LLM interaction, and looping automatically
- **Session management**: Automatic conversation history via SQLAlchemy Sessions
- **Python-first**: Use Python functions as tools directly
- **Handoffs**: Coordinate multiple agents (useful for future expansion)
- **Guardrails**: Input/output validation
- **Tracing**: Built-in visualization and debugging

## Installation

```bash
# Install OpenAI Agents SDK with SQLAlchemy support
uv add "openai-agents[sqlalchemy]"
uv add google-generativeai  # For Gemini API
uv add openai  # For AsyncOpenAI client (used by custom provider)

# Or with pip
pip install "openai-agents[sqlalchemy]"
pip install google-generativeai openai
```

## Custom Gemini Model Provider

### Creating Gemini Model Provider

Based on the [OpenAI Agents SDK custom provider example](https://github.com/openai/openai-agents-python/blob/main/examples/model_providers/custom_example_provider.py), we'll create a Gemini model provider:

```python
from __future__ import annotations
import os
from openai import AsyncOpenAI
from agents import (
    Agent,
    Model,
    ModelProvider,
    OpenAIChatCompletionsModel,
    RunConfig,
    Runner,
    set_tracing_disabled,
)
import google.generativeai as genai

# Configure Gemini
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
genai.configure(api_key=GEMINI_API_KEY)

# Gemini API endpoint (using Google's Generative AI API)
# Note: Gemini doesn't have a direct OpenAI-compatible endpoint
# We'll need to create an adapter or use Gemini's native API with a wrapper

# For now, we can use Gemini's native API and create a custom Model implementation
# Or use a service that provides OpenAI-compatible endpoint for Gemini

class GeminiModelProvider(ModelProvider):
    """
    Custom model provider that uses Gemini API instead of OpenAI.
    This requires creating a custom Model implementation that wraps Gemini.
    """
    def __init__(self, model_name: str = "gemini-2.5-flash"):
        self.model_name = model_name
        self.gemini_model = genai.GenerativeModel(model_name)

    def get_model(self, model_name: str | None) -> Model:
        # Return a custom Model that wraps Gemini API
        return GeminiChatModel(
            model=model_name or self.model_name,
            gemini_model=self.gemini_model
        )

# Custom Model implementation for Gemini
class GeminiChatModel(Model):
    """
    Custom Model implementation that wraps Gemini API
    to work with OpenAI Agents SDK.
    """
    def __init__(self, model: str, gemini_model):
        self.model_name = model
        self.gemini_model = gemini_model

    async def chat(self, messages, **kwargs):
        # Convert messages format from OpenAI to Gemini
        # Call Gemini API
        # Convert response back to OpenAI format
        # This is a simplified example - full implementation needed
        pass
```

### Using Gemini's OpenAI-Compatible Endpoint (Recommended)

Gemini provides an OpenAI-compatible API endpoint, making integration simple:

```python
from openai import AsyncOpenAI
from agents import ModelProvider, OpenAIChatCompletionsModel
import os

# Gemini OpenAI-compatible endpoint
GEMINI_BASE_URL = "https://generativelanguage.googleapis.com/v1beta/openai"
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")

# Create OpenAI client pointing to Gemini endpoint
gemini_client = AsyncOpenAI(
    base_url=GEMINI_BASE_URL,
    api_key=GEMINI_API_KEY
)

class GeminiModelProvider(ModelProvider):
    def __init__(self, model_name: str = "gemini-2.5-flash"):
        self.model_name = model_name
        self.client = gemini_client

    def get_model(self, model_name: str | None) -> Model:
        return OpenAIChatCompletionsModel(
            model=model_name or self.model_name,
            openai_client=self.client
        )
```

## Basic Agent Setup

### Creating a RAG Agent with Gemini

```python
import os
from openai import AsyncOpenAI
from agents import Agent, Runner, RunConfig, ModelProvider, OpenAIChatCompletionsModel
from agents.extensions.memory import SQLAlchemySession

# Gemini OpenAI-compatible endpoint
GEMINI_BASE_URL = "https://generativelanguage.googleapis.com/v1beta/openai"
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")

gemini_client = AsyncOpenAI(
    base_url=GEMINI_BASE_URL,
    api_key=GEMINI_API_KEY
)

# Create Gemini model provider
class GeminiModelProvider(ModelProvider):
    def __init__(self, model_name: str = "gemini-2.5-flash"):
        self.model_name = model_name
        self.client = gemini_client

    def get_model(self, model_name: str | None):
        return OpenAIChatCompletionsModel(
            model=model_name or self.model_name,
            openai_client=self.client
        )

gemini_provider = GeminiModelProvider(model_name="gemini-2.5-flash")

# Create agent with RAG instructions
agent = Agent(
    name="TextbookAssistant",
    instructions="""
    You are a helpful assistant for the Physical AI & Humanoid Robotics textbook.
    Answer questions based ONLY on the provided context from the textbook.
    If the context doesn't contain enough information, say so.
    Be concise, accurate, and cite relevant sections when possible.
    """,
    tools=[rag_tool.search_textbook]
)

# Use Gemini provider in RunConfig
result = await Runner.run(
    agent,
    query,
    session=session,
    run_config=RunConfig(model_provider=gemini_provider)
)
```

## SQLAlchemy Session Setup

### Using Neon Postgres

```python
from agents.extensions.memory import SQLAlchemySession
from sqlalchemy.ext.asyncio import create_async_engine

# Create database engine for Neon Postgres
engine = create_async_engine(
    "postgresql+asyncpg://user:password@host/database"
)

# Create session for user
session = SQLAlchemySession(
    user_id="user-123",
    engine=engine,
    create_tables=True  # Auto-create session tables
)
```

### Using Database URL

```python
# Simpler approach with database URL
session = SQLAlchemySession.from_url(
    user_id="user-123",
    url="postgresql+asyncpg://user:password@host/database",
    create_tables=True
)
```

## RAG Tool Implementation

### Creating RAG Search Tool

```python
from agents import Tool
from typing import Annotated
import qdrant_client

class RAGTool:
    def __init__(self, qdrant_client, postgres_conn):
        self.qdrant = qdrant_client
        self.postgres = postgres_conn

    @Tool
    def search_textbook(
        self,
        query: Annotated[str, "The user's question about the textbook"]
    ) -> str:
        """
        Search the textbook content for relevant information.

        Args:
            query: The question to search for

        Returns:
            Relevant context from the textbook
        """
        # Generate query embedding using Gemini
        query_embedding = self.generate_embedding(query)

        # Search in Qdrant
        results = self.qdrant.search(
            collection_name="textbook_chunks",
            query_vector=query_embedding,
            limit=5
        )

        # Retrieve chunks from Postgres
        context = self.retrieve_chunks(results)

        return context

    @Tool
    def search_selected_text(
        self,
        selected_text: Annotated[str, "The text selected by the user"],
        query: Annotated[str, "The question about the selected text"]
    ) -> str:
        """
        Answer questions based only on selected text.

        Args:
            selected_text: The text the user selected
            query: The question about the selected text

        Returns:
            Answer based on selected text only
        """
        # For selected text, use only that text as context
        # No vector search needed
        return f"Based on the selected text: {selected_text}\n\nQuestion: {query}"
```

## Running the Agent

### Async Execution

```python
import asyncio
from agents import Agent, Runner
from agents.extensions.memory import SQLAlchemySession

async def handle_query(user_id: str, query: str, selected_text: str = None):
    # Create or get session for user
    session = SQLAlchemySession.from_url(
        user_id=user_id,
        url=NEON_DATABASE_URL,
        create_tables=True
    )

    # Create agent with RAG tools
    agent = Agent(
        name="TextbookAssistant",
        instructions="...",
        tools=[rag_tool.search_textbook, rag_tool.search_selected_text]
    )

    # Build query
    if selected_text:
        user_query = f"Based on this text: {selected_text}\n\nQuestion: {query}"
    else:
        user_query = query

    # Run agent
    result = await Runner.run(
        agent,
        user_query,
        session=session
    )

    return result.final_output
```

### Sync Execution

```python
from agents import Agent, Runner

def handle_query_sync(user_id: str, query: str):
    session = SQLAlchemySession.from_url(
        user_id=user_id,
        url=NEON_DATABASE_URL
    )

    result = Runner.run_sync(
        agent,
        query,
        session=session
    )

    return result.final_output
```

## FastAPI Integration

### API Endpoint

```python
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from agents import Agent, Runner
from agents.extensions.memory import SQLAlchemySession

app = FastAPI()

class QueryRequest(BaseModel):
    query: str
    user_id: str
    selected_text: str = None
    conversation_id: str = None

@app.post("/api/chat/query")
async def query_chatbot(request: QueryRequest):
    try:
        # Get or create session
        session = SQLAlchemySession.from_url(
            user_id=request.user_id,
            url=NEON_DATABASE_URL,
            create_tables=True
        )

        # Build query
        if request.selected_text:
            user_query = f"Based on this text: {request.selected_text}\n\nQuestion: {request.query}"
        else:
            user_query = request.query

        # Run agent
        result = await Runner.run(
            agent,
            user_query,
            session=session
        )

        return {
            "answer": result.final_output,
            "conversation_id": session.session_id,
            "sources": result.tool_calls  # Get sources from tool calls
        }

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
```

## Session Management

### Automatic History

SQLAlchemySession automatically:

- Stores conversation history in database
- Retrieves history for context
- Manages session lifecycle
- Handles multiple users

### Session Lifecycle

```python
# Create session (auto-creates tables if needed)
session = SQLAlchemySession.from_url(
    user_id="user-123",
    url=DATABASE_URL,
    create_tables=True
)

# Use session - history is automatically managed
result = await Runner.run(agent, "Hello", session=session)

# Session persists across requests
result2 = await Runner.run(agent, "What did I just ask?", session=session)
# Agent has access to previous conversation
```

## Tool Context for RAG

### Building RAG Context Tool

```python
from agents import Tool
from typing import Annotated

@Tool
def get_textbook_context(
    query: Annotated[str, "The user's question"]
) -> str:
    """
    Retrieve relevant context from the textbook for answering questions.

    This tool searches the vector database and returns relevant chunks.
    """
    # 1. Generate embedding for query (using Gemini)
    embedding = generate_embedding(query)

    # 2. Search Qdrant
    results = qdrant_client.search(
        collection_name="textbook_chunks",
        query_vector=embedding,
        limit=5
    )

    # 3. Retrieve full chunks from Postgres
    chunk_ids = [r.id for r in results]
    chunks = get_chunks_from_postgres(chunk_ids)

    # 4. Combine into context
    context = "\n\n".join([chunk.content for chunk in chunks])

    return f"Context from textbook:\n{context}"

# Add tool to agent
agent = Agent(
    name="TextbookAssistant",
    instructions="Use get_textbook_context tool to find relevant information.",
    tools=[get_textbook_context]
)
```

## Selected Text Handling

### Selected Text Tool

```python
@Tool
def answer_from_selected_text(
    selected_text: Annotated[str, "The text selected by the user"],
    question: Annotated[str, "The question about the selected text"]
) -> str:
    """
    Answer questions based only on the selected text.
    Do not use any other information from the textbook.
    """
    # No vector search - use only selected text
    return f"Based on the selected text:\n{selected_text}\n\nQuestion: {question}"

# Agent automatically chooses the right tool
agent = Agent(
    tools=[get_textbook_context, answer_from_selected_text]
)
```

## Error Handling

```python
from agents.exceptions import AgentError

try:
    result = await Runner.run(agent, query, session=session)
except AgentError as e:
    # Handle agent-specific errors
    logger.error(f"Agent error: {e}")
    return {"error": "Agent processing failed"}
except Exception as e:
    # Handle other errors
    logger.error(f"Unexpected error: {e}")
    return {"error": "An error occurred"}
```

## Streaming Responses

```python
async def stream_response(agent, query, session):
    async for event in Runner.stream(agent, query, session=session):
        if event.type == "text_delta":
            yield event.content
        elif event.type == "tool_call":
            yield f"[Searching textbook...]"
```

## Best Practices

1. **Use async**: Prefer async execution for better performance
2. **Tool design**: Make tools focused and specific
3. **Error handling**: Wrap agent calls in try-except
4. **Session reuse**: Reuse sessions for same user
5. **Tool descriptions**: Write clear tool descriptions for better agent decisions

## Migration from Direct API Calls

### Before (Direct Gemini API)

```python
model = genai.GenerativeModel('gemini-2.5-flash')
response = model.generate_content(prompt)
answer = response.text
```

### After (OpenAI Agents SDK)

```python
from agents import Agent, Runner

agent = Agent(
    name="Assistant",
    instructions="...",
    tools=[rag_tool]
)

result = await Runner.run(agent, query, session=session)
answer = result.final_output
```

## Benefits

- ✅ Automatic conversation history management
- ✅ Built-in tool calling and orchestration
- ✅ Session persistence in database
- ✅ Easier to extend with more agents/tools
- ✅ Built-in tracing and debugging
- ✅ Production-ready session management

## Using Gemini as Custom Model Provider

See [GEMINI-MODEL-PROVIDER.md](./GEMINI-MODEL-PROVIDER.md) for detailed implementation of Gemini as a custom model provider.

## Resources

- [OpenAI Agents SDK Documentation](https://openai.github.io/openai-agents-python/)
- [SQLAlchemy Sessions Guide](https://openai.github.io/openai-agents-python/sessions/sqlalchemy_session/)
- [Custom Model Provider Example](https://github.com/openai/openai-agents-python/blob/main/examples/model_providers/custom_example_provider.py)
- [Agent Tools Documentation](https://openai.github.io/openai-agents-python/documentation/tools/)
- [Gemini API Documentation](https://ai.google.dev/docs)
