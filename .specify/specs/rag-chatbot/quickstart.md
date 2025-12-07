# Quick Start Guide: RAG Chatbot

This guide will help you set up and run the RAG chatbot for the Physical AI & Humanoid Robotics textbook.

## Prerequisites

1. **Python 3.11+** with UV package manager
2. **Node.js 20+** with pnpm
3. **API Keys**:
   - Google Gemini API key
   - Qdrant Cloud API key and URL
   - Neon Postgres connection string

## Step 1: Backend Setup

### 1.1 Create Backend Directory

```bash
cd physical-ai-textbook
mkdir backend
cd backend
```

### 1.2 Initialize UV Project

```bash
uv init --name rag-chatbot-backend
uv add "openai-agents[sqlalchemy]" openai fastapi uvicorn[standard] google-generativeai qdrant-client asyncpg sqlalchemy psycopg2-binary python-dotenv pydantic pydantic-settings
uv add --dev pytest pytest-asyncio httpx black ruff mypy
```

### 1.3 Environment Variables

Create `.env` file:

```bash
# OpenAI API (for Agents SDK)
OPENAI_API_KEY=your_openai_api_key_here

# Gemini API (for embeddings)
GEMINI_API_KEY=your_gemini_api_key_here

# Qdrant Cloud
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_URL=https://your-cluster.qdrant.io

# Neon Postgres (for metadata and sessions)
NEON_DATABASE_URL=postgresql+asyncpg://user:password@host/database

# Application
ENVIRONMENT=development
LOG_LEVEL=INFO
```

### 1.4 Database Setup

```bash
# Run migrations
uv run python scripts/migrate.py

# Or manually create tables (see data-model.md)
```

### 1.5 Qdrant Setup

```bash
# Create collection (will be done automatically in code)
# Or use Qdrant Cloud dashboard
```

## Step 2: Process Documents

### 2.1 Initial Processing

```bash
cd backend
uv run python scripts/process_documents.py
```

This will:

- Scan `docs/` directory for markdown files
- Parse and chunk content
- Generate embeddings using Gemini API
- Store in Qdrant and Postgres

### 2.2 Verify Processing

```bash
# Check documents in database
uv run python scripts/check_documents.py

# Verify Qdrant collection
uv run python scripts/check_qdrant.py
```

## Step 3: Start Backend Server

```bash
cd backend
uv run uvicorn src.api.main:app --reload --port 8000
```

API will be available at `http://localhost:8000`

### 3.1 Test API

```bash
# Health check
curl http://localhost:8000/api/health

# Test query
curl -X POST http://localhost:8000/api/chat/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2?", "conversation_id": null}'
```

## Step 4: Frontend Setup

### 4.1 Install Dependencies

```bash
cd physical-ai-textbook  # Root directory
pnpm install
```

### 4.2 Create Chatbot Component

```bash
# Component will be created in src/components/Chatbot/
```

### 4.3 Configure API Endpoint

Update `src/components/Chatbot/config.ts`:

```typescript
export const CHATBOT_CONFIG = {
  apiUrl:
    process.env.NODE_ENV === "production"
      ? "https://your-api-url.com/api"
      : "http://localhost:8000/api",
};
```

### 4.4 Integrate into Docusaurus

Swizzle root layout:

```bash
pnpm run swizzle @docusaurus/theme-classic Root -- --wrap
```

Add chatbot to `src/theme/Root.tsx`:

```tsx
import ChatbotWidget from "@site/src/components/Chatbot";

export default function Root({ children }) {
  return (
    <>
      {children}
      <ChatbotWidget />
    </>
  );
}
```

## Step 5: Run Development Server

### 5.1 Start Docusaurus

```bash
pnpm start
```

### 5.2 Test Chatbot

1. Open any page in the textbook
2. Look for chatbot widget in bottom-right corner
3. Click to open
4. Ask a question: "What is ROS 2?"
5. Verify answer is returned

## Step 6: Test Selected Text Query

1. Select a paragraph of text on any page
2. Chatbot should detect selection
3. Click "Ask about selected text"
4. Ask a question about the selected text
5. Verify answer uses only selected text

## Troubleshooting

### Backend Issues

**Gemini API Rate Limit**:

- Check rate limit status
- Implement rate limiting if needed
- Use caching for frequent queries

**Qdrant Connection**:

- Verify API key and URL
- Check network connectivity
- Verify collection exists

**Postgres Connection**:

- Verify connection string
- Check Neon dashboard for connection status
- Verify tables are created

### Frontend Issues

**Chatbot Not Appearing**:

- Check browser console for errors
- Verify component is imported in Root
- Check API endpoint configuration

**API Calls Failing**:

- Verify backend is running
- Check CORS configuration
- Verify API endpoint URL

## Next Steps

- Process all course content
- Test with various queries
- Optimize chunking strategy
- Add error handling
- Deploy to production

## Resources

- [Backend API Documentation](./contracts/api.yaml)
- [Data Model](./data-model.md)
- [Gemini Integration Guide](./GEMINI-INTEGRATION.md)
- [Specification](./spec.md)
