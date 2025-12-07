/**
 * TypeScript types for the chatbot component.
 */

export interface ChatMessage {
  id: string;
  role: "user" | "assistant";
  content: string;
  timestamp: Date;
}

export interface QueryRequest {
  query: string;
  user_id: string;
  selected_text?: string | null;
  session_id?: string | null;
}

export interface QueryResponse {
  answer: string;
  session_id: string;
  sources?: Array<{
    type: string;
    tool: string;
  }>;
  response_time_ms?: number;
}

export interface ChatbotProps {
  apiUrl?: string;
  userId?: string;
}
