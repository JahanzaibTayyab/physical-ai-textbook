# Quick Start Guide

## ✅ Dependencies Installed

All dependencies have been added using `uv add`, which automatically:
- Resolves correct versions
- Handles dependency conflicts
- Updates `pyproject.toml`
- Installs packages

## Next Steps

### 1. Set Environment Variables

Edit `backend/.env` and add:
- `GEMINI_API_KEY` - Your Gemini API key
- `NEON_DATABASE_URL` - Your Neon Postgres connection string

Qdrant credentials are already configured! ✅

### 2. Initialize Database

```bash
cd backend
python scripts/init_db.py
```

### 3. Process Documents

```bash
cd backend
python scripts/process_documents.py
```

This processes all markdown files and creates embeddings.

### 4. Test Connections

```bash
cd backend
python test_connection.py
```

### 5. Start Backend

```bash
cd backend
uvicorn rag_chatbot_backend.api.main:app --reload
```

### 6. Start Frontend

```bash
# In project root
pnpm start
```

### 7. Test Chatbot

- Open `http://localhost:3000`
- Click chatbot widget (bottom-right)
- Ask questions!

## Why `uv add` is Better

Instead of manually specifying versions in `pyproject.toml`:
```toml
dependencies = [
    "fastapi>=0.104.0",  # Manual version
]
```

Use `uv add`:
```bash
uv add fastapi  # Auto-resolves version
```

Benefits:
- ✅ Always gets compatible versions
- ✅ Handles dependency conflicts
- ✅ Updates lock file automatically
- ✅ Follows UV best practices

