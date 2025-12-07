# Testing Guide

## Quick Connection Test

Test all connections (Qdrant, Postgres, Gemini):

```bash
cd backend
python test_connection.py
```

## Initialize Database

Create all database tables:

```bash
cd backend
python scripts/init_db.py
```

## Process Documents

Process all markdown files and generate embeddings:

```bash
cd backend
python scripts/process_documents.py
```

This will:
1. Scan all `.md` files in `../docs/`
2. Chunk each document intelligently
3. Generate embeddings using Gemini
4. Store chunks in Postgres
5. Store vectors in Qdrant

## Test Backend API

1. Start the server:
```bash
cd backend
uvicorn rag_chatbot_backend.api.main:app --reload
```

2. Test health endpoint:
```bash
curl http://localhost:8000/health
```

3. Test query endpoint:
```bash
curl -X POST http://localhost:8000/api/chat/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is ROS 2?",
    "user_id": "test-user"
  }'
```

## Test Frontend

1. Start Docusaurus:
```bash
cd ..
pnpm start
```

2. Open browser to `http://localhost:3000`
3. Look for the chatbot widget in the bottom-right corner
4. Try asking a question

## Full End-to-End Test

1. **Setup**:
   ```bash
   # Backend
   cd backend
   python scripts/init_db.py
   python scripts/process_documents.py
   
   # Start backend
   uvicorn rag_chatbot_backend.api.main:app --reload
   ```

2. **Frontend** (in another terminal):
   ```bash
   cd ..
   pnpm start
   ```

3. **Test**:
   - Open `http://localhost:3000`
   - Click chatbot widget
   - Ask: "What is ROS 2?"
   - Verify response comes from textbook content

## Troubleshooting

### Qdrant Connection Failed
- Check `QDRANT_URL` and `QDRANT_API_KEY` in `.env`
- Verify Qdrant cluster is running

### Postgres Connection Failed
- Check `NEON_DATABASE_URL` in `.env`
- Ensure URL uses `postgresql+asyncpg://` format

### Gemini API Failed
- Check `GEMINI_API_KEY` in `.env`
- Verify API key is valid

### No Results from RAG
- Ensure documents have been processed: `python scripts/process_documents.py`
- Check Qdrant collection has vectors
- Verify Postgres has chunks

