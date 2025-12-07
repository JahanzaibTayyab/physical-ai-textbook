# Technical Research: RAG Chatbot Implementation

**Date**: 2025-01-XX  
**Feature**: RAG Chatbot for Physical AI Textbook  
**Status**: Research Phase

## Research Objectives

1. Validate Google Gemini API integration approach
2. Understand Qdrant Cloud Free Tier capabilities and limits
3. Design optimal database schema for Neon Postgres
4. Determine best chunking strategy for technical markdown content
5. Research Docusaurus React component integration patterns

---

## 1. OpenAI Agents SDK Research

### 1.1 SDK Overview

**OpenAI Agents SDK** provides:
- Built-in agent loop for tool calling and LLM interaction
- Automatic conversation history management via Sessions
- Python-first tool creation (decorate functions as tools)
- Handoffs between multiple agents
- Guardrails for input/output validation
- Built-in tracing for debugging

**Key Features**:
- ✅ Automatic session management (no manual state handling)
- ✅ SQLAlchemy Sessions for database-backed conversations
- ✅ Tool calling built-in
- ✅ Production-ready

**Installation**:
```bash
pip install "openai-agents[sqlalchemy]"
# Or with UV
uv add "openai-agents[sqlalchemy]"
```

**Documentation**: [https://openai.github.io/openai-agents-python/](https://openai.github.io/openai-agents-python/)

### 1.2 SQLAlchemy Sessions

**SQLAlchemySession** provides:
- Database-backed session storage (Postgres, MySQL, SQLite, etc.)
- Automatic conversation history management
- Multi-user support
- Session persistence across requests

**Usage**:
```python
from agents.extensions.memory import SQLAlchemySession

session = SQLAlchemySession.from_url(
    user_id="user-123",
    url="postgresql+asyncpg://...",
    create_tables=True
)
```

**Benefits**:
- ✅ No manual conversation history management
- ✅ Automatic context retrieval
- ✅ Production-ready database storage
- ✅ Works with Neon Postgres

**Documentation**: [https://openai.github.io/openai-agents-python/sessions/sqlalchemy_session/](https://openai.github.io/openai-agents-python/sessions/sqlalchemy_session/)

## 2. Google Gemini API Research (for Embeddings)

### 1.1 Embedding Models

**Available Models**:

- `models/embedding-001`: 768-dimensional embeddings
- `text-embedding-004`: Newer model (if available)

**API Usage**:

```python
import google.generativeai as genai

genai.configure(api_key=GEMINI_API_KEY)
model = genai.GenerativeModel('models/embedding-001')
result = model.embed_content("text to embed")
embedding = result['embedding']  # List of 768 floats
```

**Key Findings**:

- ✅ 768-dimensional vectors (smaller than OpenAI's 1536, but sufficient)
- ✅ Free tier: 15 RPM, 1,500 RPD
- ✅ Good for text embeddings
- ⚠️ Need to handle rate limits carefully

### 2.2 Chat Completions (Using OpenAI Agents SDK)

**OpenAI Models via Agents SDK**:

- Uses OpenAI models (GPT-4, GPT-3.5) through Agents SDK
- SDK handles model selection and configuration
- Built-in tool calling and agent loop

**API Usage**:

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

**Key Findings**:

- ✅ Automatic tool calling and orchestration
- ✅ Session management built-in
- ✅ Production-ready
- ✅ Better integration with RAG tools

### 2.3 Rate Limiting Strategy

**OpenAI API Limits**:

- Check OpenAI API tier limits
- Implement rate limiting if needed
- Use queue system for burst handling
- Cache frequent queries

**Gemini Embeddings Limits**:

- 15 requests per minute (RPM)
- 1,500 requests per day (RPD)
- Only for embeddings, not chat

---

## 2. Qdrant Cloud Free Tier Research

### 2.1 Free Tier Limits

**Collection Limits**:

- 1 collection
- 1M vectors (sufficient for our use case)
- 1GB storage

**API Limits**:

- 1 request per second (RPS)
- Sufficient for our scale (10 concurrent users)

**Key Findings**:

- ✅ 1M vectors is more than enough (we'll have ~500-1000 chunks)
- ✅ 1GB storage sufficient
- ⚠️ 1 RPS might be limiting - need efficient batching

### 2.2 Vector Search

**Search Strategy**:

- Use cosine similarity (default)
- Return top-k results (k=3-5 recommended)
- Filter by metadata if needed

**Optimization**:

- Create collection with correct vector size (768)
- Index on metadata fields (document_id, chunk_type)
- Use payload filtering for selected-text queries

---

## 3. Neon Postgres Schema Design

### 3.1 Tables Required

**documents**:

- `id` (SERIAL PRIMARY KEY)
- `file_path` (VARCHAR UNIQUE)
- `file_hash` (VARCHAR(64))
- `last_modified` (TIMESTAMP)
- `module_id` (VARCHAR)
- `embedding_status` (VARCHAR)
- `created_at`, `updated_at` (TIMESTAMP)

**chunks**:

- `id` (SERIAL PRIMARY KEY)
- `document_id` (INTEGER FOREIGN KEY)
- `chunk_index` (INTEGER)
- `content` (TEXT)
- `chunk_type` (VARCHAR) - header/paragraph/code_block
- `language_tag` (VARCHAR) - for code blocks
- `parent_section` (VARCHAR)
- `overlap_start` (INTEGER)
- `qdrant_point_id` (VARCHAR) - reference to Qdrant
- `created_at` (TIMESTAMP)

**queries** (optional, for analytics):

- `id` (SERIAL PRIMARY KEY)
- `query_text` (TEXT)
- `query_type` (VARCHAR) - general/selected
- `selected_text` (TEXT, nullable)
- `response_time_ms` (INTEGER)
- `created_at` (TIMESTAMP)

### 3.2 Indexes

- `documents.file_path` (UNIQUE)
- `documents.file_hash` (for change detection)
- `chunks.document_id` (for document queries)
- `chunks.chunk_type` (for filtering)

---

## 4. Markdown Chunking Strategy

### 4.1 Chunking Algorithm

**Priority Order**:

1. Split at markdown headers (`##`, `###`, `####`)
2. Split at paragraph boundaries (`\n\n`)
3. Split at code block boundaries (```)
4. Split at sentence boundaries (if still too large)
5. Last resort: word boundaries

**Chunk Size**:

- Target: 750 characters
- Min: 200 characters (unless complete semantic unit)
- Max: 1000 characters (hard limit)

**Overlap**:

- 100-200 characters from previous chunk
- Maintains context across boundaries

### 4.2 Code Block Handling

**Strategy**:

- Extract code blocks with language tags
- Store as separate chunks with metadata
- Never split code blocks
- Include language tag in chunk metadata

**Example**:

````markdown
## ROS 2 Nodes

Here's how to create a node:

```python
import rclpy
from rclpy.node import Node
```
````

This creates a basic node.

````

**Chunks**:
1. "## ROS 2 Nodes\n\nHere's how to create a node:"
2. Code block (language: python, preserved intact)
3. "This creates a basic node."

---

## 5. Docusaurus Integration

### 5.1 React Component Integration

**Options**:
1. **Swizzle Layout**: Modify root layout to inject component
2. **Plugin**: Create Docusaurus plugin
3. **Global Component**: Add to `src/pages/index.tsx` or create wrapper

**Recommended**: Swizzle root layout for global access

**Implementation**:
```tsx
// src/theme/Root.tsx (swizzled)
import ChatbotWidget from '@site/src/components/Chatbot';

export default function Root({children}) {
  return (
    <>
      {children}
      <ChatbotWidget />
    </>
  );
}
````

### 5.2 Text Selection API

**Browser API**: `window.getSelection()`

**Implementation**:

```typescript
const handleTextSelection = () => {
  const selection = window.getSelection();
  if (selection && selection.toString().trim()) {
    const selectedText = selection.toString();
    // Pass to chatbot
  }
};
```

**Integration**:

- Listen for `mouseup` events
- Check if text is selected
- Show "Ask about selected text" button
- Pass selected text to chatbot API

---

## 6. RAG Prompt Engineering

### 6.1 Prompt Structure

**System Prompt**:

```
You are a helpful assistant for the Physical AI & Humanoid Robotics textbook.
Answer questions based ONLY on the provided context from the textbook.
If the context doesn't contain enough information, say so.
Be concise, accurate, and cite relevant sections when possible.
```

**Context Format**:

```
Context from textbook:
[Retrieved chunk 1]
[Retrieved chunk 2]
[Retrieved chunk 3]
```

**Query Format**:

```
User: [user question]
Assistant:
```

### 6.2 Selected Text Queries

**Special Handling**:

- Use only selected text as context
- No vector search needed
- Direct prompt: "Based on the following text: [selected text]\n\nUser: [question]"

---

## 7. Performance Considerations

### 7.1 Embedding Generation

**Batch Processing**:

- Process documents in batches
- Generate embeddings in parallel (respect rate limits)
- Store in Qdrant with batch operations

**Caching**:

- Cache embeddings for unchanged files
- Use file hash to detect changes

### 7.2 Query Performance

**Optimization**:

- Pre-generate query embeddings
- Use Qdrant's efficient vector search
- Limit retrieved chunks (top 3-5)
- Cache frequent queries

**Response Time Target**: <3 seconds (p95)

---

## 8. Error Handling Strategy

### 8.1 API Failures

**Gemini API**:

- Rate limit exceeded → Queue request, retry later
- Invalid request → Return user-friendly error
- Network error → Retry with exponential backoff

**Qdrant API**:

- Connection failure → Fallback to cached results
- Search timeout → Return error, suggest retry

**Postgres**:

- Connection pool exhaustion → Queue request
- Query timeout → Return error

### 8.2 User-Facing Errors

**Error Messages**:

- "The chatbot is temporarily unavailable. Please try again in a moment."
- "I couldn't find relevant information in the textbook for that question."
- "The selected text doesn't contain enough information to answer your question."

---

## 9. Security Considerations

### 9.1 API Key Management

- Store API keys in environment variables
- Never commit keys to repository
- Use `.env.example` for documentation
- Rotate keys if exposed

### 9.2 Input Validation

- Sanitize user queries
- Limit query length (max 1000 characters)
- Validate selected text (max 5000 characters)
- Prevent injection attacks

---

## 10. Testing Strategy

### 10.1 Unit Tests

- Chunking algorithm with various markdown structures
- Embedding generation
- Vector search logic
- Prompt building

### 10.2 Integration Tests

- Full RAG flow (query → embedding → search → answer)
- Selected text queries
- Conversation history
- Error scenarios

### 10.3 End-to-End Tests

- User queries from frontend
- Text selection and query
- Error handling
- Performance benchmarks

---

## 11. Deployment Strategy

### 11.1 Backend Deployment

**Options**:

1. **Vercel Serverless Functions**: Easy, free tier
2. **Separate Service**: More control, requires hosting

**Recommended**: Vercel Serverless Functions

- Easy integration with Docusaurus
- Free tier sufficient for MVP
- Automatic scaling

### 11.2 Frontend Deployment

- Deploy Docusaurus to GitHub Pages or Vercel
- Include chatbot React component
- Configure API endpoint URL

---

## 12. Open Questions Resolved

✅ **Gemini API integration**: Validated approach
✅ **Qdrant setup**: Free tier sufficient
✅ **Database schema**: Designed and documented
✅ **Chunking strategy**: Algorithm defined
✅ **Docusaurus integration**: Approach determined
✅ **Deployment**: Strategy selected

---

## 13. Next Steps

1. Set up development environment
2. Create database schema and migrations
3. Implement core services
4. Build document processing pipeline
5. Implement RAG query flow
6. Create FastAPI endpoints
7. Build React component
8. Integration and testing
9. Deployment

---

## References

- [Google Gemini API Documentation](https://ai.google.dev/docs)
- [Qdrant Cloud Documentation](https://qdrant.tech/documentation/)
- [Neon Postgres Documentation](https://neon.tech/docs)
- [Docusaurus React Components](https://docusaurus.io/docs/markdown-features/react)
- [FastAPI Documentation](https://fastapi.tiangolo.com/)
