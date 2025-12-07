# Data Model: RAG Chatbot

**Date**: 2025-01-XX  
**Feature**: RAG Chatbot for Physical AI Textbook

## Database Schema (Neon Postgres)

### Table: `documents`

Stores metadata about markdown files from the `docs/` directory.

```sql
CREATE TABLE documents (
    id SERIAL PRIMARY KEY,
    file_path VARCHAR(500) UNIQUE NOT NULL,
    file_hash VARCHAR(64) NOT NULL,
    last_modified TIMESTAMP NOT NULL,
    module_id VARCHAR(100),
    embedding_status VARCHAR(20) DEFAULT 'pending',
    created_at TIMESTAMP DEFAULT NOW(),
    updated_at TIMESTAMP DEFAULT NOW()
);

CREATE INDEX idx_documents_file_path ON documents(file_path);
CREATE INDEX idx_documents_file_hash ON documents(file_hash);
CREATE INDEX idx_documents_module_id ON documents(module_id);
CREATE INDEX idx_documents_embedding_status ON documents(embedding_status);
```

**Fields**:

- `id`: Primary key
- `file_path`: Relative path from `docs/` directory (e.g., `module-1-ros2/intro.md`)
- `file_hash`: SHA256 hash of file content (for change detection)
- `last_modified`: File modification timestamp
- `module_id`: Module identifier (e.g., `module-1-ros2`)
- `embedding_status`: `pending`, `complete`, `needs_update`, `error`
- `created_at`, `updated_at`: Timestamps

### Table: `chunks`

Stores processed chunks of documents with metadata.

```sql
CREATE TABLE chunks (
    id SERIAL PRIMARY KEY,
    document_id INTEGER NOT NULL REFERENCES documents(id) ON DELETE CASCADE,
    chunk_index INTEGER NOT NULL,
    content TEXT NOT NULL,
    chunk_type VARCHAR(20) NOT NULL, -- 'header', 'paragraph', 'code_block'
    language_tag VARCHAR(50), -- For code blocks (e.g., 'python', 'bash')
    parent_section VARCHAR(200), -- Parent header/section
    overlap_start INTEGER DEFAULT 0, -- Start position of overlap
    qdrant_point_id VARCHAR(100) UNIQUE, -- Reference to Qdrant vector
    created_at TIMESTAMP DEFAULT NOW(),
    updated_at TIMESTAMP DEFAULT NOW(),
    UNIQUE(document_id, chunk_index)
);

CREATE INDEX idx_chunks_document_id ON chunks(document_id);
CREATE INDEX idx_chunks_chunk_type ON chunks(chunk_type);
CREATE INDEX idx_chunks_qdrant_point_id ON chunks(qdrant_point_id);
CREATE INDEX idx_chunks_parent_section ON chunks(parent_section);
```

**Fields**:

- `id`: Primary key
- `document_id`: Foreign key to documents table
- `chunk_index`: Order of chunk within document (0-based)
- `content`: The actual text content of the chunk
- `chunk_type`: Type of chunk (header, paragraph, code_block)
- `language_tag`: Programming language for code blocks
- `parent_section`: Parent section/header this chunk belongs to
- `overlap_start`: Character position where overlap from previous chunk starts
- `qdrant_point_id`: UUID or ID of corresponding vector in Qdrant
- `created_at`, `updated_at`: Timestamps

### OpenAI Agents SDK Session Tables

The SDK automatically creates session tables when using SQLAlchemySession:

```sql
-- Created automatically by SQLAlchemySession
-- Session storage for conversation history
-- Managed by OpenAI Agents SDK
```

**Note**: SQLAlchemySession handles session storage automatically. We don't need to manually create session tables.

### Table: `queries` (Optional - for analytics)

Stores query history for analytics and debugging.

```sql
CREATE TABLE queries (
    id SERIAL PRIMARY KEY,
    query_text TEXT NOT NULL,
    query_type VARCHAR(20) NOT NULL, -- 'general', 'selected'
    selected_text TEXT, -- For selected-text queries
    response_time_ms INTEGER,
    session_id VARCHAR(100), -- Reference to Agents SDK session
    created_at TIMESTAMP DEFAULT NOW()
);

CREATE INDEX idx_queries_created_at ON queries(created_at);
CREATE INDEX idx_queries_query_type ON queries(query_type);
CREATE INDEX idx_queries_session_id ON queries(session_id);
```

**Fields**:

- `id`: Primary key
- `query_text`: User's question
- `query_type`: Type of query (general or selected)
- `selected_text`: Selected text (if query_type is 'selected')
- `response_time_ms`: Response time in milliseconds
- `created_at`: Timestamp

## Vector Database Schema (Qdrant)

### Collection: `textbook_chunks`

```python
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams

client.create_collection(
    collection_name="textbook_chunks",
    vectors_config=VectorParams(
        size=768,  # Gemini embedding dimension
        distance=Distance.COSINE
    )
)
```

**Payload Structure**:

```json
{
  "chunk_id": 123,
  "document_id": 45,
  "document_path": "module-1-ros2/intro.md",
  "chunk_index": 2,
  "chunk_type": "paragraph",
  "language_tag": null,
  "parent_section": "ROS 2 Fundamentals",
  "content": "ROS 2 is a middleware framework...",
  "module_id": "module-1-ros2"
}
```

**Vector**: 768-dimensional float array (Gemini embedding)

## Data Flow

### 1. Document Processing Flow

```
Markdown File (docs/module-1-ros2/intro.md)
    ↓
Parse Markdown
    ↓
Chunk Content (500-1000 chars, with overlap)
    ↓
Generate Embeddings (Gemini API)
    ↓
Store in Qdrant (vector + payload)
    ↓
Store Metadata in Postgres (documents + chunks tables)
```

### 2. Query Flow

```
User Query
    ↓
Generate Query Embedding (Gemini API)
    ↓
Vector Search in Qdrant (top-k chunks)
    ↓
Retrieve Chunk Metadata from Postgres
    ↓
Build RAG Prompt (context + query)
    ↓
Generate Answer (Gemini Chat API)
    ↓
Return Response to User
```

### 3. Selected Text Query Flow

```
User Selects Text
    ↓
Extract Selected Text
    ↓
Build Prompt (selected text + query, NO vector search)
    ↓
Generate Answer (Gemini Chat API)
    ↓
Return Response to User
```

## Entity Relationships

```
documents (1) ──< (many) chunks
    │
    └──> module_id (string identifier)

chunks (1) ──> (1) qdrant_point (via qdrant_point_id)
```

## Data Validation Rules

### Documents

- `file_path` must be unique
- `file_hash` must be SHA256 (64 hex characters)
- `embedding_status` must be one of: `pending`, `complete`, `needs_update`, `error`

### Chunks

- `chunk_index` must be unique within a document
- `chunk_type` must be one of: `header`, `paragraph`, `code_block`
- `language_tag` required if `chunk_type` is `code_block`
- `qdrant_point_id` must be unique (one-to-one with Qdrant point)

## Migration Strategy

### Initial Migration

```sql
-- Migration: 001_create_rag_tables.sql

BEGIN;

CREATE TABLE documents (...);
CREATE TABLE chunks (...);
CREATE TABLE queries (...);

COMMIT;
```

### Future Migrations

- Add indexes for performance
- Add columns for new features
- Add constraints as needed

## Data Retention

- **Documents**: Keep all (source of truth)
- **Chunks**: Keep all (referenced by Qdrant)
- **Queries**: Optional - can be purged after 30 days for analytics

## Backup Strategy

- **Postgres**: Neon provides automatic backups
- **Qdrant**: Export collection periodically (if needed)
- **Documents**: Git repository is source of truth
