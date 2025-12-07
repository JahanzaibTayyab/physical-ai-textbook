/**
 * Text Selection Menu Component.
 *
 * Shows a pop-up menu with "Explain", "Translate", "Summarize" options
 * when user selects text anywhere on the page.
 */

import React, { useEffect, useRef, useState } from "react";

import styles from "./TextSelectionMenu.module.css";

interface TextSelectionMenuProps {
  apiUrl?: string;
}

export default function TextSelectionMenu({
  apiUrl = "http://localhost:8000",
}: TextSelectionMenuProps): React.JSX.Element {
  const [selectedText, setSelectedText] = useState<string | null>(null);
  const [menuPosition, setMenuPosition] = useState<{
    top: number;
    left: number;
  } | null>(null);
  const [isLoading, setIsLoading] = useState(false);
  const menuRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    const handleMouseUp = (e: MouseEvent) => {
      const selection = window.getSelection();
      const selectedText = selection?.toString().trim();

      if (selectedText && selectedText.length > 0) {
        // Get selection range
        const range = selection?.getRangeAt(0);
        if (range) {
          const rect = range.getBoundingClientRect();

          // Position menu above the selection
          setMenuPosition({
            top: rect.top + window.scrollY - 50,
            left: rect.left + window.scrollX + rect.width / 2,
          });
          setSelectedText(selectedText);
        }
      } else {
        setSelectedText(null);
        setMenuPosition(null);
      }
    };

    const handleClickOutside = (e: MouseEvent) => {
      if (menuRef.current && !menuRef.current.contains(e.target as Node)) {
        // Check if click is not on selected text
        const selection = window.getSelection();
        if (!selection || selection.toString().trim().length === 0) {
          setSelectedText(null);
          setMenuPosition(null);
        }
      }
    };

    document.addEventListener("mouseup", handleMouseUp);
    document.addEventListener("mousedown", handleClickOutside);

    return () => {
      document.removeEventListener("mouseup", handleMouseUp);
      document.removeEventListener("mousedown", handleClickOutside);
    };
  }, []);

  const handleAction = async (
    action: "explain" | "translate" | "summarize"
  ) => {
    if (!selectedText) return;

    setIsLoading(true);

    try {
      let message = "";

      if (action === "explain") {
        message = `Can you explain this in more detail?\n\n"${selectedText}"`;
      } else if (action === "translate") {
        // Check if user is logged in
        const userId = localStorage.getItem("user_id");
        if (!userId) {
          // Clear selection first
          setSelectedText(null);
          setMenuPosition(null);
          window.getSelection()?.removeAllRanges();
          setIsLoading(false);
          window.location.href = "/signup";
          return;
        }

        // Translate the selected text
        const response = await fetch(`${apiUrl}/api/translate/`, {
          method: "POST",
          headers: {
            "Content-Type": "application/json",
          },
          body: JSON.stringify({
            chapter_path: window.location.pathname,
            content: selectedText,
            target_language: "ur",
          }),
        });

        if (!response.ok) {
          const errorData = await response.json().catch(() => ({}));
          throw new Error(errorData.detail || "Translation failed");
        }

        const data = await response.json();
        message = `I just translated this text to Urdu. Can you help me understand it better?\n\nOriginal: "${selectedText}"\n\nTranslation: "${data.translated_content}"`;
      } else if (action === "summarize") {
        message = `Can you summarize this for me?\n\n"${selectedText}"`;
      }

      // Dispatch event to open chatbot and send message
      window.dispatchEvent(
        new CustomEvent("chatbot:send-message", {
          detail: {
            message,
            selectedText: selectedText,
          },
        })
      );

      window.dispatchEvent(new CustomEvent("chatbot:open"));

      // Clear selection
      setSelectedText(null);
      setMenuPosition(null);
      window.getSelection()?.removeAllRanges();
    } catch (err) {
      console.error("Error performing action:", err);
      const errorMessage =
        err instanceof Error
          ? err.message
          : "An error occurred. Please try again.";
      alert(errorMessage);
    } finally {
      setIsLoading(false);
    }
  };

  if (!selectedText || !menuPosition) {
    return null;
  }

  return (
    <div
      ref={menuRef}
      className={styles.menu}
      style={{
        top: `${menuPosition.top}px`,
        left: `${menuPosition.left}px`,
        transform: "translateX(-50%)",
      }}
    >
      <button
        className={`${styles.menuButton} ${isLoading ? styles.loading : ""}`}
        onClick={() => handleAction("explain")}
        disabled={isLoading}
        title="Explain this text in detail"
      >
        💡 Explain
      </button>
      <button
        className={`${styles.menuButton} ${isLoading ? styles.loading : ""}`}
        onClick={() => handleAction("translate")}
        disabled={isLoading}
        title="Translate to Urdu"
      >
        🌐 Translate
      </button>
      <button
        className={`${styles.menuButton} ${isLoading ? styles.loading : ""}`}
        onClick={() => handleAction("summarize")}
        disabled={isLoading}
        title="Summarize this text"
      >
        📝 Summarize
      </button>
    </div>
  );
}
