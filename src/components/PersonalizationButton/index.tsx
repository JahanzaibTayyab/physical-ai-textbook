/**
 * Personalization button component.
 * 
 * Displays at the start of each chapter and allows users to toggle
 * personalized content based on their background.
 */

import React, { useState, useEffect } from "react";
import styles from "./PersonalizationButton.module.css";

interface PersonalizationButtonProps {
  chapterPath: string;
  originalContent: string;
}

export default function PersonalizationButton({
  chapterPath,
  originalContent,
}: PersonalizationButtonProps): React.JSX.Element {
  const [isPersonalized, setIsPersonalized] = useState(false);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState("");
  const [personalizedContent, setPersonalizedContent] = useState("");
  const [userId, setUserId] = useState<string | null>(null);
  const [backgroundType, setBackgroundType] = useState<"software" | "hardware" | null>(null);

  useEffect(() => {
    // Check if user is logged in
    const storedUserId = localStorage.getItem("user_id");
    setUserId(storedUserId);

    if (storedUserId) {
      // Fetch user profile to determine background type
      fetch(`http://localhost:8000/api/auth/profile?user_id=${storedUserId}`)
        .then((res) => res.json())
        .then((data) => {
          if (data.software_background) {
            setBackgroundType("software");
          } else if (data.hardware_background) {
            setBackgroundType("hardware");
          }
        })
        .catch(() => {
          // User not logged in or profile not found
        });
    }
  }, []);

  const handleToggle = async () => {
    if (!userId || !backgroundType) {
      // Redirect to signup if not logged in
      window.location.href = "/signup";
      return;
    }

    if (isPersonalized) {
      // Toggle off - show original content
      setIsPersonalized(false);
      return;
    }

    // Toggle on - fetch personalized content
    setLoading(true);
    setError("");

    try {
      const response = await fetch("http://localhost:8000/api/personalize/", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({
          user_id: userId,
          chapter_path: chapterPath,
          original_content: originalContent,
          background_type: backgroundType,
        }),
      });

      if (!response.ok) {
        const data = await response.json();
        throw new Error(data.detail || "Personalization failed");
      }

      const data = await response.json();
      setPersonalizedContent(data.personalized_content);
      setIsPersonalized(true);
    } catch (err) {
      setError(err instanceof Error ? err.message : "Failed to personalize content");
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
          <span className={styles.icon}>✨</span>
          Personalize Content (Sign up required)
        </button>
      </div>
    );
  }

  return (
    <div className={styles.container}>
      <button
        className={`${styles.button} ${isPersonalized ? styles.active : ""}`}
        onClick={handleToggle}
        disabled={loading}
      >
        <span className={styles.icon}>✨</span>
        {loading
          ? "Personalizing..."
          : isPersonalized
          ? "Show Original Content"
          : `Personalize for ${backgroundType} background`}
      </button>
      {error && <div className={styles.error}>{error}</div>}
      {isPersonalized && personalizedContent && (
        <div
          className={styles.personalizedContent}
          dangerouslySetInnerHTML={{ __html: personalizedContent }}
        />
      )}
    </div>
  );
}

