# Quick Test Guide

## Fast Health Checks

### Backend Health (Should be instant)
```bash
curl --max-time 2 http://localhost:8000/health
```
Expected: `{"status":"healthy"}` in < 1 second

### Backend Root (Should be instant)
```bash
curl --max-time 2 http://localhost:8000/
```
Expected: `{"message":"Physical AI Textbook RAG Chatbot API"}` in < 1 second

## Chat Query Test (May take 5-15 seconds)

The chat query involves:
1. Creating SQLAlchemy session
2. Generating query embedding (Gemini API call)
3. Searching Qdrant vector database
4. Retrieving chunks from Postgres
5. Calling Gemini API for answer generation

**Expected time**: 5-15 seconds (depending on API response times)

```bash
curl -X POST http://localhost:8000/api/chat/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2?", "user_id": "test"}' \
  --max-time 30
```

## Performance Optimization Tips

1. **First Query**: May be slower due to:
   - Database connection initialization
   - Qdrant connection initialization
   - Gemini API cold start

2. **Subsequent Queries**: Should be faster (3-8 seconds)

3. **If Too Slow**:
   - Check Gemini API quota/rate limits
   - Verify Qdrant connection
   - Check Postgres connection
   - Monitor API response times

## Quick Status Check

```bash
# Backend status
curl --max-time 2 http://localhost:8000/health

# Frontend status  
curl --max-time 2 http://localhost:3000 | head -5
```

