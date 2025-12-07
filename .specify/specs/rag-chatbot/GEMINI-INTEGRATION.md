# Gemini API Integration Guide

This document outlines the integration of Google Gemini API for the RAG chatbot, replacing OpenAI API.

## API Usage

### 1. Embeddings (Vector Generation)

**Gemini Embedding Model**: `text-embedding-004` or `models/embedding-001`

**Implementation**:
```python
from google import generativeai as genai

genai.configure(api_key=GEMINI_API_KEY)

# Generate embedding for a text chunk
def generate_embedding(text: str) -> list[float]:
    model = genai.GenerativeModel('models/embedding-001')
    result = model.embed_content(text)
    return result['embedding']
```

**Key Differences from OpenAI**:
- Gemini embeddings are typically 768-dimensional (vs OpenAI's 1536 for text-embedding-3-small)
- API structure is different - uses `GenerativeModel` with `embed_content` method
- May need to handle rate limits differently

### 2. Chat Completions (Answer Generation)

**Gemini Model**: `gemini-2.5-flash` (recommended for speed and quality)

**Implementation**:
```python
from google import generativeai as genai

genai.configure(api_key=GEMINI_API_KEY)

def generate_answer(query: str, context: str, conversation_history: list) -> str:
    model = genai.GenerativeModel('gemini-2.5-flash')
    
    # Build prompt with context and conversation history
    prompt = build_rag_prompt(query, context, conversation_history)
    
    response = model.generate_content(prompt)
    return response.text
```

**Key Differences from OpenAI**:
- Uses `generate_content()` instead of `chat.completions.create()`
- Message format is different - uses `parts` instead of `messages` array
- Response structure: `response.text` instead of `response.choices[0].message.content`
- Context window: Gemini 2.5 Flash has large context window (check current limits)

### 3. Token Counting

**Gemini Token Estimation**:
- Gemini uses different tokenization than OpenAI
- Rough estimate: ~4 characters per token (vs OpenAI's ~3-4)
- Can use Gemini API's built-in token counting if available
- For estimation: `estimated_tokens = len(text) / 4`

**Implementation**:
```python
def estimate_tokens(text: str) -> int:
    """Estimate token count for Gemini API"""
    # Rough estimate: 4 characters per token
    return len(text) // 4

# Or use API if available
def count_tokens(model, text: str) -> int:
    """Use Gemini API to count tokens if available"""
    # Check if API provides token counting
    # Otherwise use estimation
    return estimate_tokens(text)
```

## Configuration

### Environment Variables

```bash
# .env file
GEMINI_API_KEY=your_gemini_api_key_here
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_URL=your_qdrant_url
NEON_DATABASE_URL=your_neon_postgres_url
```

### Python Dependencies (using UV)

```bash
uv add google-generativeai
uv add python-dotenv
```

### Package Installation

```bash
# Add to pyproject.toml or requirements
google-generativeai>=0.3.0
```

## RAG Prompt Structure for Gemini

```python
def build_rag_prompt(query: str, context: str, history: list[dict]) -> str:
    """Build RAG prompt for Gemini API"""
    
    system_prompt = """You are a helpful assistant for the Physical AI & Humanoid Robotics textbook.
Answer questions based ONLY on the provided context from the textbook.
If the context doesn't contain enough information, say so.
Be concise, accurate, and cite relevant sections when possible."""
    
    # Build conversation history
    history_text = ""
    for msg in history:
        role = "User" if msg["role"] == "user" else "Assistant"
        history_text += f"{role}: {msg['content']}\n"
    
    # Build full prompt
    prompt = f"""{system_prompt}

Context from textbook:
{context}

{history_text}User: {query}
Assistant:"""
    
    return prompt
```

## Error Handling

```python
from google.api_core import exceptions

try:
    response = model.generate_content(prompt)
except exceptions.ResourceExhausted:
    # Rate limit or quota exceeded
    return {"error": "API quota exceeded. Please try again later."}
except exceptions.InvalidArgument:
    # Invalid prompt or parameters
    return {"error": "Invalid request. Please rephrase your question."}
except Exception as e:
    # Other errors
    return {"error": f"An error occurred: {str(e)}"}
```

## Rate Limits and Quotas

**Gemini API Free Tier**:
- 15 requests per minute (RPM)
- 1,500 requests per day (RPD)
- Consider implementing rate limiting and caching

**Implementation**:
```python
from functools import lru_cache
import time
from collections import deque

class RateLimiter:
    def __init__(self, max_calls=15, period=60):
        self.max_calls = max_calls
        self.period = period
        self.calls = deque()
    
    def __call__(self, func):
        def wrapper(*args, **kwargs):
            now = time.time()
            # Remove calls older than period
            while self.calls and self.calls[0] < now - self.period:
                self.calls.popleft()
            
            if len(self.calls) >= self.max_calls:
                sleep_time = self.period - (now - self.calls[0])
                time.sleep(sleep_time)
            
            self.calls.append(time.time())
            return func(*args, **kwargs)
        return wrapper
```

## Migration Notes

### Changes from OpenAI to Gemini

1. **API Client**: Use `google.generativeai` instead of `openai`
2. **Embeddings**: Use `embed_content()` instead of `embeddings.create()`
3. **Chat**: Use `generate_content()` instead of `chat.completions.create()`
4. **Response Format**: Access `response.text` instead of `response.choices[0].message.content`
5. **Token Counting**: Different tokenization, use estimation or API if available
6. **Error Handling**: Use `google.api_core.exceptions` instead of `openai.error`
7. **Model Names**: Use Gemini model names (`gemini-2.5-flash`)

### Benefits of Gemini

- **Larger Context Window**: 1M-2M tokens vs OpenAI's 128K
- **Free Tier**: More generous free tier for development
- **Multimodal**: Can handle images, video (future enhancement)
- **Cost**: Potentially lower cost for high-volume usage

## Testing

```python
# Test embedding generation
def test_embedding():
    text = "ROS 2 is a middleware framework for robotics."
    embedding = generate_embedding(text)
    assert len(embedding) > 0
    assert isinstance(embedding, list)

# Test chat completion
def test_chat():
    query = "What is ROS 2?"
    context = "ROS 2 is a middleware framework..."
    response = generate_answer(query, context, [])
    assert len(response) > 0
    assert isinstance(response, str)
```

## Resources

- [Gemini API Documentation](https://ai.google.dev/docs)
- [Python SDK](https://github.com/google/generative-ai-python)
- [Gemini Models](https://ai.google.dev/models/gemini)
- [Embedding Models](https://ai.google.dev/models/embedding)

