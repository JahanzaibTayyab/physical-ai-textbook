# Embedding Optimization Guide

## Problem: Quota Exhaustion

The previous implementation was exhausting API quota because:
1. **Individual API calls**: Each chunk was processed with a separate API call
2. **Deprecated SDK**: Used old `google.generativeai` instead of new `google.genai`
3. **Deprecated model**: Used `models/embedding-001` instead of `gemini-embedding-001`
4. **No batch processing**: Processed chunks one-by-one instead of in batches

## Solution: Optimized Implementation

### Changes Made

1. **New Gemini API Client** (`google.genai`)
   - Uses official `google.genai.Client()` instead of deprecated `google.generativeai`
   - Reference: https://ai.google.dev/gemini-api/docs/embeddings

2. **Correct Model Name**
   - Changed from `models/embedding-001` to `gemini-embedding-001`
   - The old model is deprecated (October 2025)

3. **Batch Embedding Generation**
   - `generate_embeddings_batch()` processes multiple texts in a single API call
   - Reduces API calls from N (one per chunk) to 1 (batch call)
   - **Quota savings**: For 100 chunks, reduces from 100 API calls to 1 call

4. **Task Type Optimization**
   - `RETRIEVAL_DOCUMENT` for document chunks (better for storage)
   - `RETRIEVAL_QUERY` for user queries (better for search)
   - Improves embedding quality and relevance

5. **Output Dimensionality**
   - Set to 768 (recommended for RAG)
   - Can be adjusted to 1536 or 3072 if needed
   - Lower dimensions = faster processing, lower cost

### Example: Quota Impact

**Before (Old Implementation)**:
- 100 chunks = 100 API calls
- Each call counts against quota
- Quota exhausted quickly

**After (New Implementation)**:
- 100 chunks = 1 batch API call
- Single call processes all chunks
- **99% reduction in API calls**

### Usage

```python
from rag_chatbot_backend.services.embedding_service import get_embedding_service

service = get_embedding_service()

# Single embedding (for queries)
embedding = await service.generate_embedding(
    text="What is ROS 2?",
    task_type="RETRIEVAL_QUERY",
    output_dimensionality=768
)

# Batch embeddings (for documents)
embeddings = await service.generate_embeddings_batch(
    texts=["Chunk 1", "Chunk 2", "Chunk 3"],
    task_type="RETRIEVAL_DOCUMENT",
    output_dimensionality=768
)
```

### Documentation Reference

- Official Gemini Embeddings Docs: https://ai.google.dev/gemini-api/docs/embeddings
- Model: `gemini-embedding-001`
- Task Types: `RETRIEVAL_DOCUMENT`, `RETRIEVAL_QUERY`, `SEMANTIC_SIMILARITY`, etc.
- Dimensions: 128, 256, 512, 768 (recommended), 1536, 2048, 3072

