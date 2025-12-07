# Features Location Guide

## 🔐 Authentication Features

### Sign Up Page

- **URL**: `/signup`
- **File**: `src/pages/signup.tsx`
- **Features**:
  - Email/password registration
  - Software background question
  - Hardware background question
  - Auto-redirects to home after signup

### Sign In Page

- **URL**: `/signin`
- **File**: `src/pages/signin.tsx`
- **Features**:
  - Email/password authentication
  - Session management
  - Auto-redirects to home after signin

### Navigation Links

- **Location**: Top navbar (right side)
- **Links**: "Sign In" and "Sign Up" buttons
- **Config**: `docusaurus.config.ts` → `navbar.items`

---

## ✨ Personalization Features

### Personalization Button Component

- **File**: `src/components/PersonalizationButton/index.tsx`
- **Usage**: Add to chapter MDX files
- **Features**:
  - Toggles personalized content based on user background
  - Requires user to be logged in
  - Redirects to signup if not logged in

### How to Add to Chapter Pages

Add this to the top of any chapter MDX file:

```mdx
import PersonalizationButton from "@site/src/components/PersonalizationButton";

<PersonalizationButton
  chapterPath="/docs/module-1-ros2/intro"
  originalContent="Your chapter content here..."
/>
```

**Note**: The `originalContent` prop needs the full chapter content as a string. For now, you can pass a summary or key content.

---

## 🌐 Translation Features

### Translation Button Component

- **File**: `src/components/TranslationButton/index.tsx`
- **Usage**: Add to chapter MDX files
- **Features**:
  - Translates content to Urdu
  - Auto-opens chatbot with translated content
  - Requires user to be logged in

### How to Add to Chapter Pages

Add this to the top of any chapter MDX file:

```mdx
import TranslationButton from "@site/src/components/TranslationButton";

<TranslationButton
  chapterPath="/docs/module-1-ros2/intro"
  originalContent="Your chapter content here..."
/>
```

---

## 🤖 Chatbot Features

### Chatbot Widget

- **File**: `src/components/Chatbot/ChatWidget.tsx`
- **Location**: Bottom-right corner (floating widget)
- **Features**:
  - Answers questions about textbook content
  - Handles selected text queries
  - Receives translated content automatically
  - Conversation history

### Chatbot Integration

- **Auto-opens** when translation completes
- **Receives messages** from translation button
- **Shows translated content** in chat context

---

## 📍 Current Status

### ✅ Implemented

- Signup page (`/signup`)
- Signin page (`/signin`)
- PersonalizationButton component
- TranslationButton component
- Chatbot widget (always visible)
- Navigation links in navbar

### ⚠️ Needs Integration

- Personalization buttons need to be added to chapter MDX files
- Translation buttons need to be added to chapter MDX files
- User menu component (created but not added to navbar yet)

---

## 🚀 Quick Access URLs

- **Homepage**: `http://localhost:3000/`
- **Sign Up**: `http://localhost:3000/signup`
- **Sign In**: `http://localhost:3000/signin`
- **Chapter Example**: `http://localhost:3000/docs/module-1-ros2/intro`

---

## 📝 Next Steps

1. **Add buttons to all chapters**: Update all chapter MDX files to include PersonalizationButton and TranslationButton
2. **Add user menu**: Integrate UserMenu component into navbar
3. **Test full flow**: Sign up → Personalize → Translate → Chatbot

---

**Last Updated**: 2025-01-07
