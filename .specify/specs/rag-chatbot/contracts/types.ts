/**
 * TypeScript types for RAG Chatbot Frontend
 * Generated from API contract
 */

// Request Types

export interface QueryRequest {
  query: string;
  user_id: string;  // Required for session management
  session_id?: string | null;  // Optional, will be created if not provided
  max_chunks?: number;
}

export interface SelectedTextQueryRequest {
  query: string;
  selected_text: string;
  conversation_id?: string | null;
}

// Response Types

export interface Source {
  document_path: string;
  chunk_index: number;
  relevance_score: number;
}

export interface QueryResponse {
  answer: string;
  session_id: string;  // Session ID for conversation history
  sources?: Source[];
  response_time_ms?: number;
}

export interface ErrorResponse {
  error: string;
  message: string;
  code?: string;
}

// Chat Message Types

export interface ChatMessage {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
  sources?: Source[];
  error?: string;
}

export interface Conversation {
  session_id: string;  // OpenAI Agents SDK session ID
  user_id: string;
  messages: ChatMessage[];
  created_at: Date;
  updated_at: Date;
}

// Component Props

export interface ChatbotWidgetProps {
  apiUrl?: string;
  initialOpen?: boolean;
}

export interface ChatInterfaceProps {
  conversation: Conversation;
  onSendMessage: (message: string, selectedText?: string) => Promise<void>;
  isLoading: boolean;
  error: string | null;
}

export interface MessageListProps {
  messages: ChatMessage[];
}

export interface InputBoxProps {
  onSend: (message: string) => void;
  disabled?: boolean;
  placeholder?: string;
}

// API Client Types

export interface ChatbotAPIClient {
  query(request: QueryRequest): Promise<QueryResponse>;
  querySelected(request: SelectedTextQueryRequest): Promise<QueryResponse>;
  healthCheck(): Promise<{ status: string; timestamp: string }>;
}

// Configuration

export interface ChatbotConfig {
  apiUrl: string;
  maxConversationHistory: number;
  maxTokens: number;
  enableSelectedText: boolean;
}

