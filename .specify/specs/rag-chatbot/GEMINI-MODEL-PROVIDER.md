# Gemini Model Provider for OpenAI Agents SDK

This guide shows how to use Gemini API as a custom model provider in OpenAI Agents SDK, based on the [custom provider example](https://github.com/openai/openai-agents-python/blob/main/examples/model_providers/custom_example_provider.py).

## Overview

The OpenAI Agents SDK supports custom model providers, allowing us to use Gemini API instead of OpenAI's API while still benefiting from the SDK's agent orchestration, tool calling, and session management.

**Great news!** Gemini provides an OpenAI-compatible endpoint, making integration much simpler.

## Implementation Approach: Using Gemini's OpenAI-Compatible Endpoint (Recommended)

Gemini provides an OpenAI-compatible API endpoint at:

```
https://generativelanguage.googleapis.com/v1beta/openai/
```

This allows us to use `OpenAIChatCompletionsModel` directly with an `AsyncOpenAI` client pointing to Gemini's endpoint.

### Simple Implementation

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
    """Model provider using Gemini's OpenAI-compatible endpoint."""
    def __init__(self, model_name: str = "gemini-2.5-flash"):
        self.model_name = model_name
        self.client = gemini_client

    def get_model(self, model_name: str | None) -> Model:
        return OpenAIChatCompletionsModel(
            model=model_name or self.model_name,
            openai_client=self.client
        )

# Usage
GEMINI_PROVIDER = GeminiModelProvider(model_name="gemini-2.5-flash")
```

## Alternative: Direct Gemini API Integration

If the OpenAI-compatible endpoint doesn't work for your use case, you can create a fully custom Model implementation:

Since Gemini doesn't have a direct OpenAI-compatible endpoint, we'll create a custom Model implementation:

```python
from __future__ import annotations
import os
import asyncio
from typing import Any
from agents import Agent, Model, ModelProvider, RunConfig, Runner, function_tool
import google.generativeai as genai

# Configure Gemini
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
genai.configure(api_key=GEMINI_API_KEY)

class GeminiModel(Model):
    """
    Custom Model implementation that wraps Gemini API.
    """
    def __init__(self, model_name: str = "gemini-2.5-flash"):
        self.model_name = model_name
        self.gemini_model = genai.GenerativeModel(model_name)

    async def chat(self, messages: list[dict], **kwargs) -> dict:
        """
        Convert OpenAI message format to Gemini format and call API.
        """
        # Convert messages to Gemini format
        # Gemini uses a different message format
        prompt = self._convert_messages_to_prompt(messages)

        # Call Gemini API
        response = await asyncio.to_thread(
            self.gemini_model.generate_content,
            prompt
        )

        # Convert response back to OpenAI format
        return {
            "choices": [{
                "message": {
                    "role": "assistant",
                    "content": response.text
                }
            }]
        }

    def _convert_messages_to_prompt(self, messages: list[dict]) -> str:
        """Convert OpenAI message format to Gemini prompt format."""
        prompt_parts = []
        for msg in messages:
            role = msg.get("role", "user")
            content = msg.get("content", "")

            if role == "system":
                prompt_parts.append(f"System: {content}")
            elif role == "user":
                prompt_parts.append(f"User: {content}")
            elif role == "assistant":
                prompt_parts.append(f"Assistant: {content}")

        return "\n".join(prompt_parts)

class GeminiModelProvider(ModelProvider):
    """Model provider that returns Gemini models."""
    def __init__(self, model_name: str = "gemini-2.5-flash"):
        self.model_name = model_name

    def get_model(self, model_name: str | None) -> Model:
        return GeminiModel(model=model_name or self.model_name)

# Usage
GEMINI_PROVIDER = GeminiModelProvider(model_name="gemini-2.5-flash")
```

### Direct Gemini API Integration (Fallback)

If the OpenAI-compatible endpoint doesn't fully support your needs, you can create a custom Model implementation:

## Complete RAG Agent with Gemini

```python
import asyncio
import os
from openai import AsyncOpenAI
from agents import Agent, Runner, RunConfig, function_tool, ModelProvider, OpenAIChatCompletionsModel
from agents.extensions.memory import SQLAlchemySession
from typing import Annotated

# Gemini OpenAI-compatible endpoint
GEMINI_BASE_URL = "https://generativelanguage.googleapis.com/v1beta/openai"
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")

# Create OpenAI client pointing to Gemini
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

# RAG Tool
@function_tool
def search_textbook(
    query: Annotated[str, "The user's question about the textbook"]
) -> str:
    """Search the textbook content for relevant information."""
    # Generate embedding
    embedding = generate_embedding(query)  # Using Gemini embeddings

    # Search Qdrant
    results = qdrant_client.search(
        collection_name="textbook_chunks",
        query_vector=embedding,
        limit=5
    )

    # Retrieve chunks
    context = retrieve_chunks(results)
    return context

# Create agent
agent = Agent(
    name="TextbookAssistant",
    instructions="""
    You are a helpful assistant for the Physical AI & Humanoid Robotics textbook.
    Use the search_textbook tool to find relevant information before answering.
    Answer based ONLY on the retrieved context.
    """,
    tools=[search_textbook]
)

# Run with Gemini provider
async def handle_query(user_id: str, query: str):
    session = SQLAlchemySession.from_url(
        user_id=user_id,
        url=NEON_DATABASE_URL,
        create_tables=True
    )

    result = await Runner.run(
        agent,
        query,
        session=session,
        run_config=RunConfig(model_provider=gemini_provider)
    )

    return result.final_output
```

## FastAPI Integration

```python
import os
from fastapi import FastAPI
from pydantic import BaseModel
from openai import AsyncOpenAI
from agents import Agent, Runner, RunConfig, ModelProvider, OpenAIChatCompletionsModel
from agents.extensions.memory import SQLAlchemySession

app = FastAPI()

# Gemini configuration
GEMINI_BASE_URL = "https://generativelanguage.googleapis.com/v1beta/openai"
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")

gemini_client = AsyncOpenAI(
    base_url=GEMINI_BASE_URL,
    api_key=GEMINI_API_KEY
)

class GeminiModelProvider(ModelProvider):
    def __init__(self, model_name: str = "gemini-2.5-flash"):
        self.model_name = model_name
        self.client = gemini_client

    def get_model(self, model_name: str | None):
        return OpenAIChatCompletionsModel(
            model=model_name or self.model_name,
            openai_client=self.client
        )

class QueryRequest(BaseModel):
    query: str
    user_id: str
    selected_text: str = None

# Initialize Gemini provider
gemini_provider = GeminiModelProvider(model_name="gemini-2.5-flash")

@app.post("/api/chat/query")
async def query_chatbot(request: QueryRequest):
    # Create session
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

    # Run agent with Gemini
    result = await Runner.run(
        agent,
        user_query,
        session=session,
        run_config=RunConfig(model_provider=gemini_provider)
    )

    return {
        "answer": result.final_output,
        "session_id": session.session_id
    }
```

## Handling Tool Calls with Gemini

Since we're using Gemini's OpenAI-compatible endpoint, tool calling should work seamlessly with the Agents SDK. The `OpenAIChatCompletionsModel` will handle the conversion automatically.

If you encounter issues with function calling, you may need to verify:

1. The endpoint supports function calling
2. The tool schema format is compatible
3. The response format matches OpenAI's expected format

## Benefits of This Approach

1. ✅ **Use Gemini API**: Leverage Gemini's capabilities
2. ✅ **Agents SDK Features**: Still get agent orchestration, sessions, tools
3. ✅ **No OpenAI API Key Needed**: Only need Gemini API key
4. ✅ **Consistent API**: Same SDK interface regardless of model
5. ✅ **Easy to Switch**: Can switch models by changing provider

## Important Notes

- ✅ Using Gemini's OpenAI-compatible endpoint simplifies integration
- ✅ No custom Model implementation needed - use `OpenAIChatCompletionsModel`
- ✅ Tool calling should work automatically via the compatible endpoint
- ⚠️ Verify the endpoint supports all features you need (function calling, streaming, etc.)
- ⚠️ Test thoroughly to ensure compatibility with your specific use case
- ⚠️ Check Gemini API documentation for any endpoint-specific limitations

## Resources

- [OpenAI Agents SDK Custom Provider Example](https://github.com/openai/openai-agents-python/blob/main/examples/model_providers/custom_example_provider.py)
- [OpenAI Agents SDK Documentation](https://openai.github.io/openai-agents-python/)
- [Gemini API Documentation](https://ai.google.dev/docs)
- [Gemini OpenAI-Compatible Endpoint](https://generativelanguage.googleapis.com/v1beta/openai)
