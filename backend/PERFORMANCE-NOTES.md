# Performance Notes

## Expected Response Times

### Health Check
- **Target**: < 100ms
- **Actual**: Should be instant

### Chat Query
- **Target**: 3-8 seconds (p95)
- **Components**:
  1. SQLAlchemy Session creation: ~100-200ms
  2. Query embedding (Gemini API): ~500-1000ms
  3. Qdrant vector search: ~200-500ms
  4. Postgres chunk retrieval: ~100-300ms
  5. Gemini chat completion: ~2000-5000ms
  6. Total: ~3-8 seconds

## Optimization Opportunities

1. **Connection Pooling**: Already using async connections
2. **Caching**: Could cache common queries
3. **Batch Processing**: Already using batch embeddings
4. **Parallel Operations**: Embedding and DB queries could be parallel

## Troubleshooting Slow Responses

1. **Check API Quotas**: Gemini API rate limits
2. **Check Database Connections**: Postgres and Qdrant latency
3. **Check Network**: API response times
4. **Monitor Logs**: Look for errors or timeouts

## Quick Test Commands

```bash
# Health check (should be instant)
time curl -s http://localhost:8000/health

# Chat query (expect 5-15 seconds)
time curl -X POST http://localhost:8000/api/chat/query \
  -H "Content-Type: application/json" \
  -d '{"query": "test", "user_id": "test"}' \
  --max-time 30
```

