# Features Access Guide

## 🔐 Login & Authentication

### Where to Find:

1. **Top Navigation Bar** (Right Side)

   - "Sign In" link → `/signin`
   - "Sign Up" link → `/signup`

2. **Direct URLs**:
   - Sign Up: `http://localhost:3000/signup`
   - Sign In: `http://localhost:3000/signin`

### What You'll See:

- **Sign Up Page**:

  - Email and password fields
  - Software background question (optional)
  - Hardware background question (optional)
  - "Sign Up" button

- **Sign In Page**:
  - Email and password fields
  - "Sign In" button
  - Link to sign up if you don't have an account

---

## ✨ Personalized Content

### Where to Find:

- **Location**: At the top of chapter pages (after you add the component)
- **Example**: `/docs/module-1-ros2/intro` (has buttons added)

### How It Works:

1. **Sign up** with your software/hardware background
2. **Navigate** to any chapter page
3. **Click** "Personalize for [software/hardware] background" button
4. **Content** is customized based on your background
5. **Toggle** back to original content anytime

### Current Status:

- ✅ Component created: `src/components/PersonalizationButton/index.tsx`
- ✅ Added to: `docs/module-1-ros2/intro.md`
- ⚠️ **Needs to be added** to other chapter pages

---

## 🌐 Translation Feature

### Where to Find:

- **Location**: At the top of chapter pages (after you add the component)
- **Example**: `/docs/module-1-ros2/intro` (has buttons added)

### How It Works:

1. **Sign up** and log in
2. **Navigate** to any chapter page
3. **Click** "Translate to Urdu" button
4. **Content** is translated to Urdu
5. **Chatbot opens automatically** with translated content
6. **Ask questions** about the translation in the chatbot

### Current Status:

- ✅ Component created: `src/components/TranslationButton/index.tsx`
- ✅ Added to: `docs/module-1-ros2/intro.md`
- ✅ Chatbot integration working
- ⚠️ **Needs to be added** to other chapter pages

---

## 🤖 Chatbot Widget

### Where to Find:

- **Location**: Bottom-right corner (floating button)
- **Always visible** on all pages

### Features:

- Answers questions about textbook content
- Handles selected text queries
- Receives translated content automatically
- Conversation history

---

## 📍 Quick Access

### URLs:

- **Homepage**: `http://localhost:3000/`
- **Sign Up**: `http://localhost:3000/signup`
- **Sign In**: `http://localhost:3000/signin`
- **Chapter with Buttons**: `http://localhost:3000/docs/module-1-ros2/intro`

### Navigation:

- **Top Navbar**: Sign In / Sign Up links (right side)
- **Chapter Pages**: Personalization & Translation buttons (at top of content)

---

## 🚀 Testing the Features

### Test Sign Up:

1. Go to `/signup`
2. Enter email: `test@example.com`
3. Enter password: `password123`
4. Fill in software background: `Python developer, ROS 2 experience`
5. Fill in hardware background: `Robotics hardware, sensors`
6. Click "Sign Up"
7. You'll be redirected to homepage

### Test Personalization:

1. Sign in at `/signin`
2. Go to `/docs/module-1-ros2/intro`
3. Click "Personalize for software background" button
4. Content should be personalized

### Test Translation:

1. Sign in at `/signin`
2. Go to `/docs/module-1-ros2/intro`
3. Click "Translate to Urdu" button
4. Chatbot should open automatically with translated content

---

## 📝 Adding Buttons to Other Chapters

To add personalization and translation buttons to other chapter pages, add this to the top of the MDX file:

```mdx
import PersonalizationButton from "@site/src/components/PersonalizationButton";
import TranslationButton from "@site/src/components/TranslationButton";

<PersonalizationButton
  chapterPath="/docs/module-1-ros2/architecture"
  originalContent="Your chapter content summary here..."
/>

<TranslationButton
  chapterPath="/docs/module-1-ros2/architecture"
  originalContent="Your chapter content summary here..."
/>
```

**Note**: For now, pass a summary of the chapter content. In the future, we can improve this to automatically extract content.

---

**Last Updated**: 2025-01-07
