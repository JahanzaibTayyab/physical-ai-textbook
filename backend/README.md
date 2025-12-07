# RAG Chatbot Backend

Backend service for the Physical AI & Humanoid Robotics textbook RAG chatbot.

## Setup

### 1. Install Dependencies

Dependencies are managed with `uv`. They're already added to `pyproject.toml`:

```bash
uv sync
```

**Note**: Always use `uv add <package>` to add new dependencies - it automatically resolves versions and updates `pyproject.toml`.

### 2. Environment Variables

Copy `.env.example` to `.env` and configure:

```bash
cp .env.example .env
```

Required variables:
- `GEMINI_API_KEY` - Your Gemini API key
- `QDRANT_API_KEY` - Already configured ✅
- `QDRANT_URL` - Already configured ✅
- `NEON_DATABASE_URL` - Your Neon Postgres connection string

### 3. Initialize Database

```bash
uv run python scripts/init_db.py
```

### 4. Process Documents

```bash
uv run python scripts/process_documents.py
```

This processes all markdown files from `../docs/` and creates embeddings.

## Running

### Development Server

```bash
uv run uvicorn rag_chatbot_backend.api.main:app --reload
```

### Test Connections

```bash
uv run python test_connection.py
```

## API Endpoints

- `GET /` - Root endpoint
- `GET /health` - Health check
- `POST /api/chat/query` - Query the chatbot

## Project Structure

```
backend/
├── rag_chatbot_backend/
│   ├── services/          # Business logic
│   ├── models/            # Pydantic models
│   ├── database/          # SQLAlchemy models & repos
│   ├── api/               # FastAPI app
│   └── utils/             # Utilities
├── scripts/               # Processing scripts
└── pyproject.toml         # Dependencies (managed by uv)
```

## Using UV

### Add Dependencies

```bash
uv add <package-name>
```

Example:
```bash
uv add requests
```

### Add Dev Dependencies

```bash
uv add --dev <package-name>
```

### Run Commands

Always use `uv run` to ensure the virtual environment is used:

```bash
uv run python script.py
uv run uvicorn app:main
```

### Why `uv add`?

Instead of manually editing `pyproject.toml`:
- ✅ Automatically resolves compatible versions
- ✅ Handles dependency conflicts
- ✅ Updates lock file
- ✅ Follows best practices

## Testing

See `TESTING.md` for detailed testing instructions.
