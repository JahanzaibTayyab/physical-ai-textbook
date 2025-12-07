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
            👋 Ask me anything about the Physical AI & Humanoid Robotics
            textbook!
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
              <span className={styles.assistantIcon}>🤖</span>
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
            <span className={styles.assistantIcon}>🤖</span>
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
