/**
 * Message list component.
 *
 * Displays chat messages in a scrollable list.
 */

import { ChatMessage } from "./types";
import React from "react";
import styles from "./MessageList.module.css";

interface MessageListProps {
  messages: ChatMessage[];
  isLoading: boolean;
}

const MessageList: React.FC<MessageListProps> = ({ messages, isLoading }) => {
  return (
    <div className={styles.messageList}>
      {messages.length === 0 && (
        <div className={styles.emptyState}>
          <p>
            Ask me anything about the Physical AI & Humanoid Robotics textbook!
          </p>
          <p className={styles.emptyStateHint}>
            You can also select text and ask questions about it.
          </p>
        </div>
      )}
      {messages.map((message) => (
        <div
          key={message.id}
          className={`${styles.message} ${
            message.role === "user"
              ? styles.userMessage
              : styles.assistantMessage
          }`}
        >
          <div className={styles.messageContent}>
            {message.role === "assistant" && (
              <div className={styles.assistantIcon}>
                <svg
                  width="18"
                  height="18"
                  viewBox="0 0 24 24"
                  fill="none"
                  xmlns="http://www.w3.org/2000/svg"
                >
                  <path
                    d="M20 2H4C2.9 2 2 2.9 2 4V22L6 18H20C21.1 18 22 17.1 22 16V4C22 2.9 21.1 2 20 2Z"
                    stroke="currentColor"
                    strokeWidth="2"
                    strokeLinecap="round"
                    strokeLinejoin="round"
                  />
                </svg>
              </div>
            )}
            <div className={styles.messageText}>{message.content}</div>
          </div>
          <div className={styles.messageTimestamp}>
            {message.timestamp.toLocaleTimeString()}
          </div>
        </div>
      ))}
      {isLoading && (
        <div className={`${styles.message} ${styles.assistantMessage}`}>
          <div className={styles.messageContent}>
            <div className={styles.assistantIcon}>
              <svg
                width="18"
                height="18"
                viewBox="0 0 24 24"
                fill="none"
                xmlns="http://www.w3.org/2000/svg"
              >
                <path
                  d="M20 2H4C2.9 2 2 2.9 2 4V22L6 18H20C21.1 18 22 17.1 22 16V4C22 2.9 21.1 2 20 2Z"
                  stroke="currentColor"
                  strokeWidth="2"
                  strokeLinecap="round"
                  strokeLinejoin="round"
                />
              </svg>
            </div>
            <div className={styles.loadingIndicator}>
              <span></span>
              <span></span>
              <span></span>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default MessageList;
