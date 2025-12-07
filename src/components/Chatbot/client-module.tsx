/**
 * Client module for chatbot widget.
 *
 * This file is automatically loaded by Docusaurus and renders the chatbot
 * on all pages. Docusaurus will only execute this in the browser.
 */

import ChatWidget from "./ChatWidget";
import React from "react";
import { createRoot } from "react-dom/client";

// Client modules execute as side effects
if (typeof window !== "undefined") {
  const initChatbot = () => {
    // Check if already initialized
    if (document.getElementById("chatbot-widget-root")) {
      return;
    }

    // Create root container
    const root = document.createElement("div");
    root.id = "chatbot-widget-root";
    document.body.appendChild(root);

    // Render the chatbot
    const reactRoot = createRoot(root);
    reactRoot.render(
      React.createElement(ChatWidget, {
        apiUrl: "http://localhost:8000",
        userId: localStorage.getItem("userId") || "anonymous",
      })
    );
  };

  // Initialize when DOM is ready
  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", initChatbot);
  } else {
    setTimeout(initChatbot, 100);
  }
}

// Default export required by Docusaurus (can be empty)
export default function ChatbotClientModule() {
  return null;
}
