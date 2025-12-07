# Implementation Plan: RAG Chatbot for Physical AI Textbook

**Branch**: `001-rag-chatbot` | **Date**: 2025-01-XX | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `.specify/specs/rag-chatbot/spec.md`

## Summary

Build a RAG (Retrieval-Augmented Generation) chatbot that allows users to ask questions about the Physical AI & Humanoid Robotics textbook content. The system uses Google Gemini API for embeddings and chat completions, Qdrant Cloud for vector storage, and Neon Postgres for metadata. The chatbot is embedded as a floating React widget in Docusaurus pages and supports both general book queries and selected-text-only queries.

**Technical Approach**:

- FastAPI backend with Python 3.11 (UV package manager)
- Google Gemini API for embeddings (`models/embedding-001`)
- OpenAI Agents SDK with Gemini API as custom model provider for chat completions (`gemini-2.5-flash`)
- SQLAlchemy Sessions (from OpenAI Agents SDK) for conversation management using Neon Postgres
- Qdrant Cloud Free Tier for vector similarity search
- Neon Serverless Postgres for document/chunk metadata and session storage
- React component (floating widget) integrated into Docusaurus
- Markdown content processing with intelligent chunking (500-1000 chars, respecting structure)

## Technical Context

**Language/Version**: Python 3.11  
**Primary Dependencies**: FastAPI, openai-agents[sqlalchemy], google-generativeai, qdrant-client, psycopg2-binary, pydantic, sqlalchemy  
**Storage**:

- Vector: Qdrant Cloud Free Tier
- Relational: Neon Serverless Postgres
- Files: Markdown files in `docs/` directory  
  **Testing**: pytest, pytest-asyncio, httpx (for API testing)  
  **Target Platform**:
- Backend: Linux server (Vercel serverless functions or separate service)
- Frontend: Web browser (Docusaurus static site)
  **Project Type**: Web application (frontend + backend)  
  **Performance Goals**:
- Query response time: <3 seconds (p95 latency)
- Chatbot widget load: <1 second
- Support 10 concurrent users
  **Constraints**:
- Must work within free tier limits:
  - Gemini API: 15 RPM, 1,500 RPD
  - Qdrant Cloud Free Tier limits
  - Neon Postgres free tier limits
- Embedding dimension: 768 (Gemini embeddings)
- Context window: 1M-2M tokens (Gemini 2.5)
  **Scale/Scope**:
- ~50-100 markdown files in `docs/` directory
- Estimated 500-1000 chunks after processing
- 10 concurrent users initially
- Single textbook deployment

## Constitution Check

_GATE: Must pass before Phase 0 research. Re-check after Phase 1 design._

✅ **I. Spec-Driven Development**: Specification complete with user stories, requirements, and success criteria
✅ **II. AI-Native Development**: Using Spec-Kit Plus workflow, Gemini API integration
✅ **III. Test-First Development**: Test requirements defined in spec, will be implemented
✅ **IV. Documentation as Code**: All content in Markdown, API docs will be maintained
✅ **V. User-Centric Design**: Floating widget design, selected text support, conversation history

**Constitution Compliance**: ✅ PASSED

## Project Structure

### Documentation (this feature)

```text
.specify/specs/rag-chatbot/
├── spec.md              # Feature specification ✅
├── plan.md              # This file ✅
├── research.md          # Technical research (to be created)
├── data-model.md        # Database schema design (to be created)
├── quickstart.md        # Quick start guide (to be created)
├── contracts/           # API contracts (to be created)
│   ├── api.yaml        # OpenAPI/Swagger spec
│   └── types.ts        # TypeScript types for frontend
├── CLARIFICATIONS.md    # Resolved open questions ✅
├── GEMINI-INTEGRATION.md # Gemini API integration guide ✅
└── SUMMARY.md           # Specification summary ✅
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── api/
│   │   ├── __init__.py
│   │   ├── main.py              # FastAPI app entry point
│   │   ├── routes/
│   │   │   ├── __init__.py
│   │   │   ├── chat.py          # Chat endpoints
│   │   │   ├── embeddings.py    # Embedding management endpoints
│   │   │   └── health.py        # Health check endpoint
│   │   └── middleware/
│   │       ├── __init__.py
│   │       └── rate_limiter.py  # Rate limiting for Gemini API
│   ├── services/
│   │   ├── __init__.py
│   │   ├── embedding_service.py # Gemini embeddings
│   │   ├── chat_service.py      # Gemini chat completions
│   │   ├── vector_service.py    # Qdrant operations
│   │   ├── document_service.py  # Markdown processing
│   │   └── chunking_service.py  # Intelligent chunking
│   ├── models/
│   │   ├── __init__.py
│   │   ├── document.py          # Document model
│   │   ├── chunk.py             # Chunk model
│   │   ├── query.py             # Query model
│   │   └── conversation.py      # Conversation model
│   ├── database/
│   │   ├── __init__.py
│   │   ├── connection.py        # Postgres connection
│   │   ├── repositories/
│   │   │   ├── __init__.py
│   │   │   ├── document_repo.py
│   │   │   ├── chunk_repo.py
│   │   │   └── query_repo.py
│   │   └── migrations/          # Database migrations
│   └── utils/
│       ├── __init__.py
│       ├── markdown_parser.py  # Markdown parsing utilities
│       └── token_counter.py     # Token estimation for Gemini
├── tests/
│   ├── unit/
│   │   ├── test_chunking.py
│   │   ├── test_embedding.py
│   │   └── test_chat.py
│   ├── integration/
│   │   ├── test_api.py
│   │   └── test_rag_flow.py
│   └── fixtures/
│       └── sample_markdown.md
├── scripts/
│   ├── process_documents.py     # Initial document processing
│   └── update_embeddings.py     # Incremental update script
├── pyproject.toml               # UV project config
├── .env.example                 # Environment variables template
└── README.md                    # Backend README

frontend/ (Docusaurus project root)
├── src/
│   ├── components/
│   │   ├── Chatbot/
│   │   │   ├── index.tsx        # Main chatbot component
│   │   │   ├── ChatWidget.tsx   # Floating widget
│   │   │   ├── ChatInterface.tsx # Chat UI
│   │   │   ├── MessageList.tsx   # Message display
│   │   │   ├── InputBox.tsx      # Query input
│   │   │   └── styles.module.css
│   │   └── TextSelector/
│   │       ├── index.tsx         # Text selection handler
│   │       └── styles.module.css
│   └── hooks/
│       ├── useChatbot.ts         # Chatbot state management
│       └── useTextSelection.ts  # Text selection hook
├── static/
│   └── chatbot-config.json       # Chatbot configuration
└── docusaurus.config.ts          # Docusaurus config (add chatbot plugin)
```

**Structure Decision**: Web application structure (Option 2) - separate `backend/` and frontend (Docusaurus root). This allows:

- Independent deployment of backend API
- Frontend can be deployed to GitHub Pages/Vercel
- Clear separation of concerns
- Backend can be scaled independently

## Implementation Phases

### Phase 0: Research & Setup

**Research Areas**:

1. Google Gemini API integration patterns

   - Embedding generation best practices
   - Chat completion with context
   - Rate limiting strategies
   - Error handling

2. Qdrant Cloud Free Tier

   - Collection setup and configuration
   - Vector search optimization
   - Free tier limits and constraints

3. Neon Postgres

   - Connection pooling for serverless
   - Schema design for documents/chunks
   - Migration strategy

4. Markdown Processing

   - Parsing markdown with code blocks
   - Chunking strategies for technical content
   - Handling MDX files

5. Docusaurus Integration
   - React component injection
   - Floating widget implementation
   - Text selection API

**Deliverables**:

- `research.md` with findings
- Technology stack validation
- API key setup instructions

### Phase 1: Backend Foundation

**1.1 Project Setup**

- Initialize UV project in `backend/` directory
- Create `pyproject.toml` with dependencies
- Set up environment variable management
- Create `.env.example` template

**1.2 Database Schema**

- Design Postgres schema (documents, chunks, queries tables)
- Create migration files
- Set up database connection with Neon

**1.3 Qdrant Setup**

- Create Qdrant collection
- Configure vector size (768 for Gemini embeddings)
- Set up connection client

**1.4 Core Services**

- `embedding_service.py`: Gemini embedding generation
- `vector_service.py`: Qdrant operations (store, search)
- `document_service.py`: Markdown file processing
- `chunking_service.py`: Intelligent chunking logic
- `gemini_model_provider.py`: Custom Gemini model provider for Agents SDK

**Deliverables**:

- `data-model.md` with schema design
- Database migrations
- Core service implementations
- Unit tests for services

### Phase 2: Document Processing Pipeline

**2.1 Markdown Parser**

- Parse markdown files from `docs/` directory
- Extract code blocks with language tags
- Handle headers, paragraphs, lists

**2.2 Chunking Implementation**

- Implement chunking strategy (500-1000 chars, overlap)
- Respect markdown structure
- Preserve code blocks intact

**2.3 Embedding Generation**

- Generate embeddings for all chunks using Gemini
- Store embeddings in Qdrant
- Store metadata in Postgres

**2.4 Incremental Updates**

- File hash tracking
- Change detection
- Selective regeneration

**Deliverables**:

- Document processing script
- Chunking service with tests
- Initial embedding generation
- Update script for incremental changes

### Phase 3: RAG Query Pipeline

**3.1 Query Processing**

- Accept user queries (general or selected-text)
- Create or retrieve SQLAlchemy session for user
- Generate query embedding using Gemini (for general queries)
- Vector similarity search in Qdrant (for general queries)

**3.2 Context Retrieval**

- Retrieve top-k relevant chunks from Qdrant
- For selected-text queries: use only selected text (no vector search)
- Combine retrieved context
- Pass context to RAG tool

**3.3 Answer Generation**

- Build RAG prompt with context
- Use OpenAI Agents SDK with Gemini custom model provider
- Create agent with RAG tools
- Agent automatically handles tool calling and response generation via Gemini
- Format and return response

**3.4 Conversation Management**

- Use OpenAI Agents SDK SQLAlchemySession for automatic conversation history
- Sessions stored in Neon Postgres via SQLAlchemy
- SDK handles token counting and context management automatically

**Deliverables**:

- Gemini model provider implementation
- Chat service using Agents SDK
- RAG tool implementation
- RAG query endpoint
- SQLAlchemy session integration
- Integration tests

### Phase 4: FastAPI Backend API

**4.1 API Endpoints**

- `POST /api/chat/query` - General book query
- `POST /api/chat/query-selected` - Selected text query
- `GET /api/health` - Health check
- `POST /api/embeddings/process` - Process documents (admin)
- `POST /api/embeddings/update` - Update embeddings (admin)

**4.2 Error Handling**

- API error responses
- Rate limiting middleware
- Validation with Pydantic

**4.3 API Documentation**

- OpenAPI/Swagger spec
- Type definitions
- Example requests/responses

**Deliverables**:

- FastAPI application
- All API endpoints
- Error handling
- API documentation

### Phase 5: Frontend React Component

**5.1 Chatbot Widget**

- Floating widget component
- Toggle open/close
- Responsive design

**5.2 Chat Interface**

- Message list display
- Input box with send button
- Loading states
- Error messages

**5.3 Text Selection**

- Detect user text selection
- Pass selected text to chatbot
- Visual indicator for selected text

**5.4 API Integration**

- Frontend API client
- Conversation state management
- Error handling

**Deliverables**:

- React chatbot component
- Docusaurus integration
- Text selection functionality
- Frontend tests

### Phase 6: Integration & Testing

**6.1 End-to-End Testing**

- Full RAG flow testing
- Selected text query testing
- Error scenario testing

**6.2 Performance Testing**

- Response time measurement
- Concurrent user testing
- Rate limit testing

**6.3 Deployment Preparation**

- Environment configuration
- Deployment scripts
- Documentation

**Deliverables**:

- Integration tests
- Performance benchmarks
- Deployment guide
- User documentation

## Complexity Tracking

> **No violations detected** - Architecture follows constitution principles

## Risk Assessment

| Risk                    | Impact | Mitigation                                     |
| ----------------------- | ------ | ---------------------------------------------- |
| Gemini API rate limits  | High   | Implement rate limiting, caching, queue system |
| Qdrant free tier limits | Medium | Monitor usage, optimize vector storage         |
| Large markdown files    | Medium | Efficient chunking, streaming processing       |
| Selected text accuracy  | Medium | Careful prompt engineering, validation         |
| Deployment complexity   | Low    | Use Vercel serverless functions, clear docs    |

## Dependencies & Prerequisites

### External Services

- ✅ Google Gemini API key
- ✅ Qdrant Cloud account and API key
- ✅ Neon Postgres connection string
- ⏳ Docusaurus project (already set up)

### Development Tools

- ✅ Python 3.11
- ✅ UV package manager
- ✅ Node.js and pnpm
- ✅ Git

## Next Steps

1. ✅ Specification complete
2. ✅ Implementation plan complete (this file)
3. ⏭️ Create research document (`research.md`)
4. ⏭️ Create data model (`data-model.md`)
5. ⏭️ Create API contracts (`contracts/`)
6. ⏭️ Break down into tasks (`/sp.tasks rag-chatbot`)
7. ⏭️ Implement (`/sp.implement rag-chatbot`)
