/**
 * Floating chatbot widget component.
 *
 * Displays as a toggleable widget in the bottom-right corner.
 */

import React, { useEffect, useRef, useState } from "react";

import ChatInterface from "./ChatInterface";
import styles from "./ChatWidget.module.css";

interface ChatWidgetProps {
  apiUrl?: string;
  userId?: string;
}

const ChatWidget: React.FC<ChatWidgetProps> = ({
  apiUrl = "http://localhost:8000",
  userId = "anonymous",
}) => {
  const [isOpen, setIsOpen] = useState(false);
  const [isMinimized, setIsMinimized] = useState(false);
  const widgetRef = useRef<HTMLDivElement>(null);

  const toggleWidget = () => {
    setIsOpen(!isOpen);
    setIsMinimized(false);
  };

  const minimizeWidget = () => {
    setIsMinimized(true);
  };

  // Close widget when clicking outside
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (
        widgetRef.current &&
        !widgetRef.current.contains(event.target as Node) &&
        isOpen &&
        !isMinimized
      ) {
        // Don't close on outside click - let user explicitly close
      }
    };

    document.addEventListener("mousedown", handleClickOutside);
    return () => {
      document.removeEventListener("mousedown", handleClickOutside);
    };
  }, [isOpen, isMinimized]);

  return (
    <div className={styles.chatWidgetContainer} ref={widgetRef}>
      {isOpen && !isMinimized && (
        <div className={styles.chatWidget}>
          <div className={styles.chatWidgetHeader}>
            <h3 className={styles.chatWidgetTitle}>📚 Textbook Assistant</h3>
            <div className={styles.chatWidgetActions}>
              <button
                className={styles.minimizeButton}
                onClick={minimizeWidget}
                aria-label="Minimize"
              >
                −
              </button>
              <button
                className={styles.closeButton}
                onClick={toggleWidget}
                aria-label="Close"
              >
                ×
              </button>
            </div>
          </div>
          <ChatInterface apiUrl={apiUrl} userId={userId} />
        </div>
      )}
      {isOpen && isMinimized && (
        <div className={styles.minimizedWidget}>
          <button
            className={styles.expandButton}
            onClick={() => setIsMinimized(false)}
            aria-label="Expand"
          >
            📚
          </button>
        </div>
      )}
      {!isOpen && (
        <button
          className={styles.toggleButton}
          onClick={toggleWidget}
          aria-label="Open chatbot"
        >
          💬
        </button>
      )}
    </div>
  );
};

export default ChatWidget;
