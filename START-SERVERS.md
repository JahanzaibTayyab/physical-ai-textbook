# How to Start and Test Backend & Frontend

## Quick Start

### Terminal 1: Start Backend
```bash
cd /Users/zaib/Panaverse/hackathon-1/physical-ai-textbook/backend
uv run uvicorn rag_chatbot_backend.api.main:app --reload
```

Wait for: `Uvicorn running on http://127.0.0.1:8000`

### Terminal 2: Start Frontend
```bash
cd /Users/zaib/Panaverse/hackathon-1/physical-ai-textbook
pnpm start
```

Wait for: `Local: http://localhost:3000`

## Quick Tests

### Test Backend Health (should be instant)
```bash
curl http://localhost:8000/health
```
Expected: `{"status":"healthy"}`

### Test Backend Chat (takes 5-15 seconds)
```bash
curl -X POST http://localhost:8000/api/chat/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2?", "user_id": "test"}'
```

### Test Frontend
1. Open browser: `http://localhost:3000`
2. Look for chatbot widget (bottom-right)
3. Click to open
4. Ask a question

## Why Chat Queries Take Time

The chat query involves multiple steps:
1. **SQLAlchemy Session**: ~100ms
2. **Gemini Embedding API**: ~500-1000ms  
3. **Qdrant Vector Search**: ~200-500ms
4. **Postgres Chunk Retrieval**: ~100-300ms
5. **Gemini Chat API**: ~2000-5000ms

**Total**: 5-15 seconds (normal for RAG systems)

## Troubleshooting

### Backend won't start
- Check `.env` file has all required variables
- Check port 8000 is not in use
- Look for error messages in terminal

### Frontend won't start  
- Check port 3000 is not in use
- Run `pnpm install` if needed
- Check for build errors

### Chat queries timeout
- Check Gemini API key is valid
- Check API quota/rate limits
- Verify Qdrant and Postgres connections

## Status

✅ **All code implemented**  
✅ **All data loaded** (200 embeddings)  
✅ **Ready for testing**

Start the servers and test in your browser!

