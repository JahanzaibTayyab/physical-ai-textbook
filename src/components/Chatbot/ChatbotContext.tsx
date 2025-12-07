/**
 * Chatbot context for cross-component communication.
 * 
 * Allows other components to send messages to the chatbot programmatically.
 */

import React, { createContext, useContext, useState, useCallback, ReactNode } from "react";

interface ChatbotContextType {
  sendMessage: (message: string, selectedText?: string) => void;
  openChatbot: () => void;
  isOpen: boolean;
  setIsOpen: (open: boolean) => void;
}

const ChatbotContext = createContext<ChatbotContextType | undefined>(undefined);

export const useChatbot = () => {
  const context = useContext(ChatbotContext);
  if (!context) {
    throw new Error("useChatbot must be used within ChatbotProvider");
  }
  return context;
};

interface ChatbotProviderProps {
  children: ReactNode;
  onSendMessage?: (message: string, selectedText?: string) => void;
  onOpen?: () => void;
}

export const ChatbotProvider: React.FC<ChatbotProviderProps> = ({
  children,
  onSendMessage,
  onOpen,
}) => {
  const [isOpen, setIsOpen] = useState(false);

  const sendMessage = useCallback(
    (message: string, selectedText?: string) => {
      if (onSendMessage) {
        onSendMessage(message, selectedText);
      }
      // Also dispatch a custom event for components that can't use context
      window.dispatchEvent(
        new CustomEvent("chatbot:send-message", {
          detail: { message, selectedText },
        })
      );
    },
    [onSendMessage]
  );

  const openChatbot = useCallback(() => {
    setIsOpen(true);
    if (onOpen) {
      onOpen();
    }
    // Dispatch custom event
    window.dispatchEvent(new CustomEvent("chatbot:open"));
  }, [onOpen]);

  return (
    <ChatbotContext.Provider value={{ sendMessage, openChatbot, isOpen, setIsOpen }}>
      {children}
    </ChatbotContext.Provider>
  );
};

