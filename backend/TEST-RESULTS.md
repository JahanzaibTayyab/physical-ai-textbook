# Backend & Frontend Test Results

## Backend Tests

### Health Check

```bash
curl http://localhost:8000/health
```

**Expected**: `{"status":"ok"}`

### Root Endpoint

```bash
curl http://localhost:8000/
```

**Expected**: `{"message":"Physical AI Textbook RAG Chatbot API is running!"}`

### Chat Query Test

```bash
curl -X POST http://localhost:8000/api/chat/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2?", "user_id": "test-user-123"}'
```

**Expected**: JSON response with `response` field containing answer about ROS 2

### Selected Text Query Test

```bash
curl -X POST http://localhost:8000/api/chat/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Explain this concept",
    "user_id": "test-user-123",
    "selected_text": "ROS 2 is a flexible framework for writing robot software."
  }'
```

**Expected**: JSON response based only on selected text

## Frontend Tests

### Docusaurus Build

```bash
cd physical-ai-textbook
pnpm run build
```

**Expected**: Successful build with no errors

### Development Server

```bash
pnpm start
```

**Expected**: Server starts on `http://localhost:3000`

### Chatbot Widget

1. Open `http://localhost:3000` in browser
2. Look for chatbot widget in bottom-right corner
3. Click to open chat interface
4. Type a question and verify response

## End-to-End Test Flow

1. **Start Backend**:

   ```bash
   cd backend
   uv run uvicorn rag_chatbot_backend.api.main:app --reload
   ```

2. **Start Frontend** (in another terminal):

   ```bash
   cd physical-ai-textbook
   pnpm start
   ```

3. **Test in Browser**:

   - Navigate to `http://localhost:3000`
   - Open chatbot widget
   - Ask: "What is ROS 2?"
   - Verify answer comes from textbook content

4. **Test Selected Text**:
   - Select text on any page
   - Ask question about selected text
   - Verify answer uses only selected text

## Expected Results

- ✅ Backend responds to health checks
- ✅ Backend processes chat queries
- ✅ Frontend loads successfully
- ✅ Chatbot widget appears on all pages
- ✅ Queries return relevant answers
- ✅ Selected text queries work correctly
