/**
 * Translation button component.
 * 
 * Displays at the start of each chapter and allows users to toggle
 * between English and Urdu translations.
 */

import React, { useState, useEffect } from "react";
import styles from "./TranslationButton.module.css";

interface TranslationButtonProps {
  chapterPath: string;
  originalContent: string;
}

export default function TranslationButton({
  chapterPath,
  originalContent,
}: TranslationButtonProps): React.JSX.Element {
  const [isTranslated, setIsTranslated] = useState(false);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState("");
  const [translatedContent, setTranslatedContent] = useState("");
  const [userId, setUserId] = useState<string | null>(null);

  useEffect(() => {
    // Check if user is logged in
    const storedUserId = localStorage.getItem("user_id");
    setUserId(storedUserId);
  }, []);

  const handleToggle = async () => {
    if (!userId) {
      // Redirect to signup if not logged in
      window.location.href = "/signup";
      return;
    }

    if (isTranslated) {
      // Toggle off - show original content
      setIsTranslated(false);
      return;
    }

    // Toggle on - fetch translated content
    setLoading(true);
    setError("");

    try {
      const response = await fetch("http://localhost:8000/api/translate/", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({
          chapter_path: chapterPath,
          content: originalContent,
          target_language: "ur",
        }),
      });

      if (!response.ok) {
        const data = await response.json();
        throw new Error(data.detail || "Translation failed");
      }

      const data = await response.json();
      setTranslatedContent(data.translated_content);
      setIsTranslated(true);
    } catch (err) {
      setError(err instanceof Error ? err.message : "Failed to translate content");
    } finally {
      setLoading(false);
    }
  };

  if (!userId) {
    return (
      <div className={styles.container}>
        <button
          className={styles.button}
          onClick={() => (window.location.href = "/signup")}
        >
          <span className={styles.icon}>🌐</span>
          Translate to Urdu (Sign up required)
        </button>
      </div>
    );
  }

  return (
    <div className={styles.container}>
      <button
        className={`${styles.button} ${isTranslated ? styles.active : ""}`}
        onClick={handleToggle}
        disabled={loading}
      >
        <span className={styles.icon}>🌐</span>
        {loading
          ? "Translating..."
          : isTranslated
          ? "Show English"
          : "Translate to Urdu"}
      </button>
      {error && <div className={styles.error}>{error}</div>}
      {/* Note: Translated content should replace the chapter content, not be displayed here */}
      {/* This component is meant to be used in MDX files where content replacement happens */}
    </div>
  );
}

