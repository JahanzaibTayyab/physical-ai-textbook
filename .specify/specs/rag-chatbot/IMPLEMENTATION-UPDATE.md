# RAG Chatbot Implementation Update: OpenAI Agents SDK

## Summary of Changes

The RAG chatbot implementation has been updated to use **OpenAI Agents SDK** with **Gemini API as a custom model provider**, along with **SQLAlchemy Sessions** for conversation management. This allows us to use Gemini for both embeddings and chat completions while leveraging the SDK's powerful features.

## Key Changes

### 1. Chat Completions

**Before**: Direct Gemini API calls

```python
model = genai.GenerativeModel('gemini-2.5-flash')
response = model.generate_content(prompt)
```

**After**: OpenAI Agents SDK with Gemini Custom Provider

```python
from agents import Agent, Runner, RunConfig
from custom_provider import GeminiModelProvider

# Create Gemini model provider
gemini_provider = GeminiModelProvider(model_name="gemini-2.5-flash")

agent = Agent(
    name="TextbookAssistant",
    instructions="...",
    tools=[rag_tool]
)

# Use Gemini via custom provider
result = await Runner.run(
    agent,
    query,
    session=session,
    run_config=RunConfig(model_provider=gemini_provider)
)
```

### 2. Session Management

**Before**: Manual conversation history management

```python
conversation_history = []  # Manual list
# Manual trimming, token counting, etc.
```

**After**: SQLAlchemy Sessions (automatic)

```python
from agents.extensions.memory import SQLAlchemySession

session = SQLAlchemySession.from_url(
    user_id="user-123",
    url=NEON_DATABASE_URL,
    create_tables=True
)
# History automatically managed by SDK
```

### 3. Tool Integration

**Before**: Manual tool calling

```python
# Manual prompt building with tool results
prompt = build_rag_prompt(query, context, history)
```

**After**: Built-in tool system

```python
@Tool
def search_textbook(query: str) -> str:
    # Tool automatically called by agent
    return context

agent = Agent(tools=[search_textbook])
# Agent automatically calls tools as needed
```

## Benefits

1. **Automatic Session Management**: No manual conversation history handling
2. **Built-in Tool Calling**: Agent automatically decides when to use tools
3. **Database-Backed Sessions**: Persistent conversations in Postgres
4. **Production-Ready**: SDK handles edge cases and errors
5. **Easier to Extend**: Add more agents/tools easily
6. **Built-in Tracing**: Debug and monitor workflows

## Updated Architecture

```
User Query
    ↓
OpenAI Agents SDK Agent
    ↓
RAG Tool (searches Qdrant)
    ↓
Retrieves Context
    ↓
Agent generates answer
    ↓
SQLAlchemySession stores in Postgres
    ↓
Return response
```

## Dependencies Updated

```bash
# New dependencies
uv add "openai-agents[sqlalchemy]"
uv add openai  # For AsyncOpenAI client (used in custom provider pattern)
uv add asyncpg  # For async Postgres
uv add sqlalchemy

# Existing (still needed)
uv add google-generativeai  # For embeddings and chat (via custom provider)
uv add qdrant-client
uv add fastapi
```

## Environment Variables

```bash
# Gemini API (for both embeddings and chat completions)
GEMINI_API_KEY=your_gemini_api_key

# Note: No OPENAI_API_KEY needed - using Gemini via custom model provider

# Existing
QDRANT_API_KEY=...
NEON_DATABASE_URL=postgresql+asyncpg://...
```

## Implementation Files to Update

1. ✅ Specification updated
2. ✅ Implementation plan updated
3. ✅ Research document updated
4. ✅ Data model updated
5. ✅ API contracts updated
6. ✅ Quickstart guide updated
7. ✅ Dependencies template updated
8. ⏭️ Backend implementation (next step)

## Next Steps

1. Update backend service implementations
2. Create RAG tool using OpenAI Agents SDK
3. Integrate SQLAlchemy Sessions
4. Update FastAPI endpoints
5. Test agent with RAG tools

## Resources

- [OpenAI Agents SDK](https://openai.github.io/openai-agents-python/)
- [SQLAlchemy Sessions](https://openai.github.io/openai-agents-python/sessions/sqlalchemy_session/)
- [Custom Model Provider Example](https://github.com/openai/openai-agents-python/blob/main/examples/model_providers/custom_example_provider.py)
- [Agent Tools](https://openai.github.io/openai-agents-python/documentation/tools/)
- [Gemini API Documentation](https://ai.google.dev/docs)
