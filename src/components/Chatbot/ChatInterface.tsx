/**
 * Chat interface component.
 *
 * Handles message display and user input.
 */

import { ChatMessage, QueryRequest, QueryResponse } from "./types";
import React, { useEffect, useRef, useState, useImperativeHandle, forwardRef } from "react";

import InputBox from "./InputBox";
import MessageList from "./MessageList";
import styles from "./ChatInterface.module.css";

interface ChatInterfaceProps {
  apiUrl: string;
  userId: string;
}

export interface ChatInterfaceRef {
  sendMessage: (message: string, selectedText?: string) => void;
}

const ChatInterface = forwardRef<ChatInterfaceRef, ChatInterfaceProps>(
  ({ apiUrl, userId }, ref) => {
    const [messages, setMessages] = useState<ChatMessage[]>([]);
    const [isLoading, setIsLoading] = useState(false);
    const [sessionId, setSessionId] = useState<string | null>(null);
    const [selectedText, setSelectedText] = useState<string | null>(null);
    const messagesEndRef = useRef<HTMLDivElement>(null);

    // Scroll to bottom when messages change
    useEffect(() => {
      messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
    }, [messages]);

    // Handle text selection
    useEffect(() => {
      const handleSelection = () => {
        const selection = window.getSelection();
        if (selection && selection.toString().trim()) {
          setSelectedText(selection.toString().trim());
        } else {
          setSelectedText(null);
        }
      };

      document.addEventListener("selectionchange", handleSelection);
      return () => {
        document.removeEventListener("selectionchange", handleSelection);
      };
    }, []);

    const handleSendMessage = async (query: string, providedSelectedText?: string) => {
      if (!query.trim() || isLoading) return;

      const textToUse = providedSelectedText || selectedText;

      // Add user message
      const userMessage: ChatMessage = {
        id: Date.now().toString(),
        role: "user",
        content: query,
        timestamp: new Date(),
      };
      setMessages((prev) => [...prev, userMessage]);
      setIsLoading(true);

      try {
        const request: QueryRequest = {
          query,
          user_id: userId,
          selected_text: textToUse || null,
          session_id: sessionId,
        };

        const response = await fetch(`${apiUrl}/api/chat/query`, {
          method: "POST",
          headers: {
            "Content-Type": "application/json",
          },
          body: JSON.stringify(request),
        });

        if (!response.ok) {
          throw new Error(`HTTP error! status: ${response.status}`);
        }

        const data: QueryResponse = await response.json();

        // Update session ID
        if (data.session_id) {
          setSessionId(data.session_id);
        }

        // Add assistant message
        const assistantMessage: ChatMessage = {
          id: (Date.now() + 1).toString(),
          role: "assistant",
          content: data.answer,
          timestamp: new Date(),
        };
        setMessages((prev) => [...prev, assistantMessage]);

        // Clear selected text after use
        if (!providedSelectedText) {
          setSelectedText(null);
        }
      } catch (error) {
        console.error("Error sending message:", error);
        const errorMessage: ChatMessage = {
          id: (Date.now() + 1).toString(),
          role: "assistant",
          content: "Sorry, I encountered an error. Please try again.",
          timestamp: new Date(),
        };
        setMessages((prev) => [...prev, errorMessage]);
      } finally {
        setIsLoading(false);
      }
    };

    // Expose sendMessage method via ref
    useImperativeHandle(ref, () => ({
      sendMessage: (message: string, selectedText?: string) => {
        handleSendMessage(message, selectedText);
      },
    }), [apiUrl, userId, sessionId, selectedText, isLoading]);

    return (
      <div className={styles.chatInterface}>
        {selectedText && (
          <div className={styles.selectedTextBanner}>
            <span>Answering about selected text</span>
            <button
              className={styles.clearSelectionButton}
              onClick={() => setSelectedText(null)}
            >
              Clear
            </button>
          </div>
        )}
        <MessageList messages={messages} isLoading={isLoading} />
        <InputBox onSendMessage={(msg) => handleSendMessage(msg)} disabled={isLoading} />
      </div>
    );
  }
);

ChatInterface.displayName = "ChatInterface";

export default ChatInterface;
