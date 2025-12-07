# Translation-Chatbot Integration

## Overview

When a user translates content using the Translation button, the translated content is automatically sent to the AI chatbot widget, which opens and provides a response about the translation.

## How It Works

1. **User clicks "Translate to Urdu" button** on a chapter page
2. **Translation API** generates Urdu translation
3. **Translation button** dispatches a custom event with the translated content
4. **Chatbot widget** listens for the event and:
   - Opens automatically
   - Sends a message asking about the translation
   - Includes the translated content as selected text for context

## Implementation Details

### Event System

The integration uses browser custom events for cross-component communication:

```typescript
// Translation button dispatches event
window.dispatchEvent(
  new CustomEvent("chatbot:send-message", {
    detail: {
      message: "I just translated this content to Urdu. Can you help me understand it better?",
      selectedText: translatedContent,
    },
  })
);

// Chatbot widget listens for event
window.addEventListener("chatbot:send-message", handleSendMessage);
```

### Components Modified

1. **TranslationButton** (`src/components/TranslationButton/index.tsx`)
   - Dispatches `chatbot:send-message` event after successful translation
   - Includes translated content in event detail

2. **ChatWidget** (`src/components/Chatbot/ChatWidget.tsx`)
   - Listens for `chatbot:send-message` and `chatbot:open` events
   - Opens widget and sends message automatically

3. **ChatInterface** (`src/components/Chatbot/ChatInterface.tsx`)
   - Exposes `sendMessage` method via ref
   - Accepts optional `selectedText` parameter for context

## User Experience

1. User clicks "Translate to Urdu" button
2. Translation completes
3. Success message appears: "✓ Content translated! The chatbot has been opened with the translated content."
4. Chatbot widget opens automatically
5. A message is sent: "I just translated this content to Urdu. Can you help me understand it better?"
6. The translated content is included as context
7. AI responds with helpful information about the translation

## Benefits

- **Seamless workflow**: No need to manually copy/paste translated content
- **Context-aware**: Chatbot receives full translated content for better responses
- **User-friendly**: Automatic chatbot opening reduces friction
- **Educational**: Users can immediately ask questions about translations

## Future Enhancements

- Add similar integration for personalization feature
- Allow users to ask specific questions about translations
- Support multiple languages beyond Urdu
- Add translation quality feedback

