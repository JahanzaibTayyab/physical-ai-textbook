/**
 * Input box component.
 *
 * Handles user input and message sending.
 */

import React, { KeyboardEvent, useState } from "react";

import styles from "./InputBox.module.css";

interface InputBoxProps {
  onSendMessage: (message: string) => void;
  disabled?: boolean;
}

const InputBox: React.FC<InputBoxProps> = ({ onSendMessage, disabled }) => {
  const [input, setInput] = useState("");

  const handleSend = () => {
    if (input.trim() && !disabled) {
      onSendMessage(input.trim());
      setInput("");
    }
  };

  const handleKeyPress = (e: KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === "Enter" && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  return (
    <div className={styles.inputBox}>
      <textarea
        className={styles.input}
        value={input}
        onChange={(e) => setInput(e.target.value)}
        onKeyPress={handleKeyPress}
        placeholder="Ask a question about the textbook..."
        disabled={disabled}
        rows={1}
      />
      <button
        className={styles.sendButton}
        onClick={handleSend}
        disabled={disabled || !input.trim()}
        aria-label="Send message"
      >
        ➤
      </button>
    </div>
  );
};

export default InputBox;
