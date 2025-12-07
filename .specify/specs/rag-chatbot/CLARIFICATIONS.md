# RAG Chatbot Specification - Clarifications

This document provides detailed clarifications for the open questions in the RAG chatbot specification.

## 1. Chatbot UI Placement

**Decision**: Floating widget accessible on all pages

**Details**:
- **Location**: Bottom-right corner of the viewport
- **Default State**: Minimized (shows as a chat icon/button)
- **Expanded State**: Opens a chat interface overlay (modal or slide-up panel)
- **Accessibility**: Available on all Docusaurus pages without requiring page modifications
- **Implementation**: 
  - React component that can be added to Docusaurus root layout
  - Or use Docusaurus plugin system to inject into all pages
  - Responsive design for mobile and desktop

**Benefits**:
- ✅ Meets requirement: "embedded within the published book"
- ✅ Better UX: Always accessible, doesn't require modifying every markdown file
- ✅ Non-intrusive: Can be minimized when not needed
- ✅ Consistent experience across all pages

## 2. Conversation History Management

**Decision**: Last 10 messages OR 4000 tokens, whichever is reached first

**Details**:
- **Storage**: Maintain conversation in memory (session-based) or localStorage for persistence
- **Trimming Strategy**: When limit is reached, remove oldest messages first
- **Token Counting**: Use Gemini API's token counting or estimate based on character count (Gemini uses different tokenization)
- **Context Window**: Include system prompt + conversation history + retrieved context + current query

**Implementation**:
```python
# Pseudo-code
MAX_MESSAGES = 10
MAX_TOKENS = 4000  # Gemini context window limit (adjust based on model)

def trim_conversation(messages):
    if len(messages) > MAX_MESSAGES:
        messages = messages[-MAX_MESSAGES:]
    
    # Estimate tokens (Gemini: ~4 chars per token, or use API if available)
    total_tokens = estimate_tokens(messages)  # or use Gemini API token counting
    if total_tokens > MAX_TOKENS:
        # Remove oldest messages until under limit
        while estimate_tokens(messages) > MAX_TOKENS and len(messages) > 1:
            messages.pop(0)
    
    return messages
```

## 3. Embedding Regeneration Strategy

**Decision**: Incremental updates with full regeneration capability

**Details**:
- **Initial Setup**: Process all markdown files in `docs/` directory
- **Change Detection**: Track file hash (MD5/SHA256) and modification time
- **Incremental Updates**: 
  - On content update, compare file hash with stored hash
  - If changed, regenerate embeddings only for that file
  - Delete old chunks for that file, insert new chunks
- **Full Regeneration**: Provide admin script/endpoint for complete rebuild

**Database Schema**:
```sql
CREATE TABLE documents (
    id SERIAL PRIMARY KEY,
    file_path VARCHAR(500) UNIQUE NOT NULL,
    file_hash VARCHAR(64) NOT NULL,
    last_modified TIMESTAMP NOT NULL,
    embedding_status VARCHAR(20) DEFAULT 'pending',
    created_at TIMESTAMP DEFAULT NOW(),
    updated_at TIMESTAMP DEFAULT NOW()
);
```

**Update Process**:
1. Scan `docs/` directory for markdown files
2. Calculate file hash
3. Check against database
4. If hash changed or file is new:
   - Delete old chunks from Qdrant
   - Process file: chunk → embed → store
   - Update document record

## 4. Code Block Handling

**Decision**: Preserve code blocks intact with language tags

**Details**:
- **Extraction**: Parse markdown, identify code blocks with language tags
- **Storage**: 
  - Option A: Store code blocks as separate chunks with metadata
  - Option B: Include code blocks in regular chunks but mark them
- **Chunking**: Never split code blocks across chunks
- **Metadata**: Store language tag, code block index, parent section

**Example**:
```markdown
## ROS 2 Nodes

Here's how to create a node:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
```

This creates a basic node.
```

**Chunking Strategy**:
- Chunk 1: "## ROS 2 Nodes\n\nHere's how to create a node:"
- Chunk 2: Code block (preserved with language tag `python`)
- Chunk 3: "This creates a basic node."

**Metadata**:
```json
{
  "chunk_type": "code_block",
  "language": "python",
  "code_content": "import rclpy\n...",
  "parent_section": "ROS 2 Nodes"
}
```

## 5. Chunk Size Strategy

**Decision**: 500-1000 characters with 100-200 character overlap, respect markdown structure

**Details**:
- **Target Size**: 750 characters (middle of range)
- **Minimum**: 200 characters (unless it's a complete semantic unit)
- **Maximum**: 1000 characters (hard limit)
- **Overlap**: 100-200 characters from previous chunk

**Chunking Priority**:
1. **Primary**: Split at markdown headers (`##`, `###`, `####`)
2. **Secondary**: Split at paragraph boundaries (`\n\n`)
3. **Tertiary**: Split at code block boundaries
4. **Quaternary**: Split at sentence boundaries (if still too large)
5. **Last Resort**: Split at word boundaries (avoid if possible)

**Overlap Strategy**:
- Include last 100-200 characters of previous chunk
- Helps maintain context across chunk boundaries
- Improves retrieval quality for queries spanning chunks

**Example**:
```
Chunk 1 (750 chars): "...ROS 2 is a middleware framework. It provides..."
Chunk 2 (750 chars): "...provides communication mechanisms. Nodes are..." [overlap: "provides communication mechanisms"]
```

**Implementation**:
```python
def chunk_markdown(content, target_size=750, overlap=150):
    chunks = []
    
    # Split by headers first
    sections = split_by_headers(content)
    
    for section in sections:
        if len(section) <= target_size:
            chunks.append(section)
        else:
            # Split by paragraphs
            paragraphs = split_by_paragraphs(section)
            current_chunk = ""
            
            for para in paragraphs:
                if len(current_chunk + para) <= target_size:
                    current_chunk += para
                else:
                    if current_chunk:
                        chunks.append(current_chunk)
                        # Add overlap
                        current_chunk = current_chunk[-overlap:] + para
                    else:
                        # Paragraph too large, split by sentences
                        chunks.extend(split_large_paragraph(para, target_size))
    
    return chunks
```

## Summary

All open questions have been resolved with clear decisions and implementation strategies. The specification is now ready for the planning phase.

