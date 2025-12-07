# RAG Chatbot Specification Summary

## ✅ Specification Status: COMPLETE

All open questions have been resolved and the specification is ready for implementation planning.

## Key Decisions Made

### 1. **AI Service**: Google Gemini API
- **Embeddings**: Gemini embedding models (`models/embedding-001` or `text-embedding-004`)
- **Chat Completions**: Gemini 2.5 Flash (`gemini-2.5-flash` recommended for speed)
- **Benefits**: 
  - Larger context window (1M-2M tokens)
  - Free tier: 15 RPM, 1,500 RPD
  - Potentially lower costs
  - Multimodal capabilities (future enhancement)

### 2. **UI Placement**: Floating Widget
- Bottom-right corner, toggleable
- Available on all pages
- Non-intrusive design

### 3. **Conversation History**: 10 messages OR 4000 tokens
- Balances context with API costs
- Efficient token management

### 4. **Embedding Strategy**: Incremental Updates
- Track file hashes for change detection
- Regenerate only changed files
- Support full regeneration when needed

### 5. **Code Block Handling**: Preserve Intact
- Keep code blocks with language tags
- Never split across chunks
- Store as separate chunks or with metadata

### 6. **Chunking Strategy**: 500-1000 chars with overlap
- Respect markdown structure
- 100-200 character overlap
- Priority: Headers → Paragraphs → Code blocks → Sentences

## Updated Requirements

- **FR-005**: Use Google Gemini embeddings API
- **FR-006**: Use Google Gemini API for chat completions
- **FR-016**: Preserve code blocks intact with language tags
- **FR-017**: Support incremental embedding updates
- **FR-018**: Maintain conversation history (10 messages OR 4000 tokens)

## Technical Stack

- **Backend**: FastAPI (Python 3.11), UV package manager
- **AI**: Google Gemini API (`google-generativeai` package)
- **Vector DB**: Qdrant Cloud Free Tier
- **Database**: Neon Serverless Postgres
- **Frontend**: React component for Docusaurus

## Next Steps

1. ✅ Specification complete
2. ⏭️ Create implementation plan (`/sp.plan rag-chatbot`)
3. ⏭️ Break down into tasks (`/sp.tasks rag-chatbot`)
4. ⏭️ Implement (`/sp.implement rag-chatbot`)

## Documentation

- **Spec**: `.specify/specs/rag-chatbot/spec.md`
- **Clarifications**: `.specify/specs/rag-chatbot/CLARIFICATIONS.md`
- **Gemini Integration**: `.specify/specs/rag-chatbot/GEMINI-INTEGRATION.md`
- **Summary**: `.specify/specs/rag-chatbot/SUMMARY.md` (this file)

## Dependencies to Install

```bash
# Python dependencies (using UV)
uv add google-generativeai
uv add fastapi
uv add qdrant-client
uv add psycopg2-binary
uv add python-dotenv
uv add pydantic
uv add pydantic-settings
```

## Environment Variables Needed

```bash
GEMINI_API_KEY=your_gemini_api_key
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_URL=your_qdrant_url
NEON_DATABASE_URL=your_neon_postgres_url
```

