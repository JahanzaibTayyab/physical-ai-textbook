# Feature Specification: RAG Chatbot for Physical AI Textbook

**Feature Branch**: `001-rag-chatbot`  
**Created**: 2025-01-XX  
**Status**: Draft  
**Input**: User description: "Build a RAG (Retrieval-Augmented Generation) chatbot for the Physical AI & Humanoid Robotics textbook using OpenAI Agents SDK for chat completions and Google Gemini API for embeddings"

## User Scenarios & Testing _(mandatory)_

### User Story 1 - General Book Content Query (Priority: P1)

A student reading the textbook wants to ask questions about the course content to better understand concepts. They open any chapter page and see a chatbot widget. They type a question like "What is ROS 2?" and receive an accurate answer based on the textbook content.

**Why this priority**: This is the core functionality required for the base score (100 points). Without this, the RAG chatbot feature is incomplete.

**Independent Test**: Can be fully tested by asking a question about any topic covered in the textbook and verifying the answer is accurate and sourced from the book content. Delivers immediate value to students.

**Acceptance Scenarios**:

1. **Given** a user is on any textbook page, **When** they type "What is ROS 2?" in the chatbot, **Then** they receive an accurate answer explaining ROS 2 based on Module 1 content
2. **Given** a user asks "How does Gazebo simulate physics?", **When** the chatbot processes the query, **Then** it retrieves relevant content from Module 2 and provides a comprehensive answer
3. **Given** a user asks about a concept not in the book, **When** the chatbot processes the query, **Then** it responds that it can only answer questions about the Physical AI & Humanoid Robotics course content

---

### User Story 2 - Selected Text Query (Priority: P1)

A student highlights a specific paragraph or section in the textbook and wants to ask a question about only that selected text. They select text, click the chatbot, and ask a question that should be answered based solely on the selected content.

**Why this priority**: This is explicitly required in the requirements ("Answer questions based on user-selected text only"). It's a core differentiator and required for base score.

**Independent Test**: Can be fully tested by selecting any text passage, asking a question about it, and verifying the answer only uses information from the selected text, not the entire book. Delivers precise, context-specific answers.

**Acceptance Scenarios**:

1. **Given** a user selects a paragraph about "ROS 2 Nodes", **When** they ask "How do I create a node?", **Then** the answer is based only on the selected text, not the entire Module 1 content
2. **Given** a user selects text about "NVIDIA Isaac Sim", **When** they ask "What are the system requirements?", **Then** the chatbot searches only within the selected text for relevant information
3. **Given** a user selects text and asks a question that cannot be answered from that text, **When** the chatbot processes it, **Then** it responds that the selected text doesn't contain enough information to answer

---

### User Story 3 - Chat History and Context (Priority: P2)

A user engages in a multi-turn conversation with the chatbot, asking follow-up questions. The chatbot maintains conversation context and can reference previous messages in the conversation.

**Why this priority**: Enhances user experience significantly but is not strictly required for base functionality. Can be added after core P1 features work.

**Independent Test**: Can be tested by having a conversation with multiple questions and verifying follow-up questions are answered with context from previous messages.

**Acceptance Scenarios**:

1. **Given** a user asks "What is ROS 2?", **When** they follow up with "How do I install it?", **Then** the chatbot understands "it" refers to ROS 2
2. **Given** a conversation about Module 1 topics, **When** the user asks "Can you give me an example?", **Then** the chatbot provides an example relevant to the current conversation context

---

### User Story 4 - Error Handling and Loading States (Priority: P2)

The chatbot handles errors gracefully (API failures, network issues, invalid queries) and shows appropriate loading states during processing.

**Why this priority**: Essential for production quality but not blocking for MVP. Users need feedback when things go wrong.

**Independent Test**: Can be tested by simulating API failures, network disconnections, and invalid inputs to verify appropriate error messages are shown.

**Acceptance Scenarios**:

1. **Given** the Gemini API is unavailable, **When** a user asks a question, **Then** they see a user-friendly error message and are informed to try again later
2. **Given** a user submits a query, **When** the chatbot is processing, **Then** a loading indicator is shown
3. **Given** a user submits an empty query, **When** they click send, **Then** they receive validation feedback to enter a question

---

### Edge Cases

- What happens when the vector database (Qdrant) is unavailable?
- How does the system handle very long queries (e.g., >1000 characters)?
- What happens when no relevant content is found for a query?
- How does the system handle special characters, code snippets, or mathematical formulas in queries?
- What happens when a user selects text that contains code blocks or images?
- How does the system handle concurrent queries from multiple users?
- What happens when the embedding generation fails?
- How does the system handle queries in languages other than English?

## Requirements _(mandatory)_

### Functional Requirements

- **FR-001**: System MUST allow users to ask questions about textbook content from any page
- **FR-002**: System MUST support answering questions based on user-selected text only
- **FR-003**: System MUST embed the chatbot as a floating React widget accessible on all Docusaurus pages (bottom-right corner, toggleable)
- **FR-004**: System MUST process all markdown content from the `docs/` directory for embedding
- **FR-005**: System MUST use Google Gemini embeddings API for vector generation
- **FR-006**: System MUST use OpenAI Agents SDK with Gemini API as custom model provider for generating chat completions/answers
- **FR-019**: System MUST use OpenAI Agents SDK with SQLAlchemy Sessions for conversation management
- **FR-007**: System MUST store embeddings in Qdrant Cloud Free Tier vector database
- **FR-008**: System MUST store metadata (document IDs, chunk IDs, timestamps) in Neon Serverless Postgres
- **FR-009**: System MUST provide a FastAPI backend API for chatbot operations
- **FR-010**: System MUST implement proper error handling for API failures, network issues, and invalid inputs
- **FR-011**: System MUST show loading states during query processing
- **FR-012**: System MUST chunk markdown content appropriately for embedding (500-1000 chars per chunk, 100-200 char overlap, respect markdown structure - headers, paragraphs, code blocks)
- **FR-016**: System MUST preserve code blocks intact with language tags during chunking
- **FR-017**: System MUST support incremental embedding updates (regenerate only changed files based on modification time/hash)
- **FR-018**: System MUST maintain conversation history (last 10 messages OR 4000 tokens, whichever is reached first)
- **FR-013**: System MUST support both general book queries and selected-text-only queries
- **FR-014**: System MUST maintain conversation history within a session
- **FR-015**: System MUST use Python 3.11 and UV package manager for backend

### Key Entities _(include if feature involves data)_

- **Document**: Represents a markdown file from the `docs/` directory. Attributes: file_path, content, last_modified, file_hash, module_id, embedding_status (pending/complete/needs_update)
- **Chunk**: Represents a processed segment of a document for embedding. Attributes: chunk_id, document_id, content, embedding_vector, chunk_index, chunk_type (header/paragraph/code_block), language_tag (for code blocks), parent_section, overlap_start
- **Query**: Represents a user question. Attributes: query_id, user_id (optional), query_text, selected_text (optional), timestamp, query_type (general/selected)
- **Conversation**: Represents a chat session. Attributes: conversation_id (session_id), messages (array), created_at, updated_at. Managed by OpenAI Agents SDK SQLAlchemySession
- **Embedding**: Represents a vector embedding. Attributes: embedding_id, chunk_id, vector, model_version

## Success Criteria _(mandatory)_

### Measurable Outcomes

- **SC-001**: Users can ask questions and receive accurate answers within 3 seconds (p95 latency)
- **SC-002**: System correctly answers questions about textbook content with >85% accuracy (verified against source material)
- **SC-003**: Selected text queries return answers based solely on selected content (100% of selected-text queries must not reference non-selected content)
- **SC-004**: Chatbot widget loads and is interactive within 1 second of page load
- **SC-005**: System handles 10 concurrent users without degradation
- **SC-006**: All markdown files in `docs/` directory are successfully processed and embedded
- **SC-007**: Error rate for Gemini API calls is <1% under normal conditions (accounting for free tier rate limits)
- **SC-008**: Users can successfully complete a query-answer cycle 95% of the time on first attempt

## Technical Constraints

- **Backend**: FastAPI (Python 3.11), UV package manager
- **Vector Database**: Qdrant Cloud Free Tier
- **Relational Database**: Neon Serverless Postgres (for document metadata and SQLAlchemy Sessions)
- **AI Services**:
  - Google Gemini API (embeddings and chat completions via custom model provider)
  - OpenAI Agents SDK (agent orchestration, tool calling, session management)
- **Frontend**: React component compatible with Docusaurus
- **Deployment**: Backend must be deployable (Vercel serverless functions or separate service)
- **Cost**: Must work within free tier limits of Qdrant, Neon, and Gemini API (15 RPM, 1,500 RPD)

## Non-Goals

- User authentication (handled in separate bonus feature)
- Multi-language support beyond English (Urdu translation is separate bonus feature)
- Voice input (not required for base functionality)
- Offline functionality
- Mobile app version
- Integration with external knowledge bases beyond the textbook

## Dependencies

- Google Gemini API key (required, for embeddings and chat completions)
- OpenAI Agents SDK (no OpenAI API key needed - using Gemini via custom model provider)
- Qdrant Cloud account and API key (required)
- Neon Postgres connection string (required)
- Access to `docs/` directory for content processing
- Docusaurus build process for content updates

## Open Questions - RESOLVED

### ✅ Chatbot UI Placement

**Decision**: Floating widget that appears on all pages
**Rationale**:

- Requirements state "embedded within the published book" - a floating widget meets this requirement
- Better UX: accessible from any page without modifying every markdown file
- Can be toggled open/closed to not obstruct reading
- Implementation: React component that can be added to Docusaurus layout or as a plugin
  **Implementation**: Create a floating chat button/widget in bottom-right corner, expandable to show chat interface

### ✅ Maximum Conversation History Length

**Decision**: Last 10 messages OR 4000 tokens, whichever is reached first
**Rationale**:

- Balances context preservation with API cost and performance
- Gemini 2.5 models have large context windows (1M-2M tokens), but we should be efficient
- 10 messages provides good conversation continuity for follow-up questions
- 4000 token limit prevents excessive API costs and stays within free tier limits
  **Implementation**: Maintain conversation buffer, trim oldest messages when limit exceeded

### ✅ Embedding Regeneration Strategy

**Decision**: Support both initial full embedding and incremental updates
**Rationale**:

- Textbook content may be updated during development
- Full regeneration on every change is inefficient
- Need to track file modification times and regenerate only changed files
  **Implementation**:
- Initial setup: Process all markdown files in `docs/` directory
- Updates: Track file hashes/modification times, regenerate embeddings only for changed files
- Provide admin endpoint or script to trigger full regeneration if needed
- Store file metadata (path, hash, last_modified) in Postgres for change detection

### ✅ Code Block Handling in Embeddings

**Decision**: Preserve code blocks with language tags, chunk at code block boundaries when possible
**Rationale**:

- Code examples are critical for technical textbook content
- Preserving language tags helps with context understanding
- Code blocks should be kept intact (not split) to maintain meaning
  **Implementation**:
- Extract code blocks with their language tags (e.g., `python`, `bash`, `yaml`)
- Store code blocks as separate chunks or mark them with special metadata
- When chunking, prefer splitting at paragraph/code block boundaries rather than mid-code
- Include code block language in chunk metadata for better retrieval

### ✅ Chunk Size Strategy

**Decision**: 500-1000 characters per chunk with 100-200 character overlap, respect markdown structure
**Rationale**:

- 500-1000 chars provides good balance: enough context, not too large
- Overlap prevents losing context at chunk boundaries
- Respecting markdown structure (headers, paragraphs, code blocks) maintains semantic meaning
  **Implementation**:
- Primary chunking: Split at markdown headers (##, ###) and paragraph boundaries
- Secondary: If chunk too small, combine with next paragraph (max 1000 chars)
- Tertiary: If still too large, split at sentence boundaries (max 1000 chars)
- Overlap: Include last 100-200 chars of previous chunk at start of next chunk
- Store chunk metadata: source_file, chunk_index, chunk_type (header, paragraph, code_block), parent_section
