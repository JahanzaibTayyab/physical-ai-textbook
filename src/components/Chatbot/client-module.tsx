/**
 * Client module for chatbot widget.
 *
 * This file is automatically loaded by Docusaurus and renders the chatbot
 * on all pages. Docusaurus will only execute this in the browser.
 */

import ChatWidget from "./ChatWidget";
import React from "react";

export default function ChatbotClientModule() {
  // Only render in browser
  if (typeof window === "undefined") {
    return null;
  }

  return (
    <ChatWidget
      apiUrl={process.env.REACT_APP_CHATBOT_API_URL || "http://localhost:8000"}
      userId={
        typeof window !== "undefined"
          ? localStorage.getItem("userId") || "anonymous"
          : "anonymous"
      }
    />
  );
}
