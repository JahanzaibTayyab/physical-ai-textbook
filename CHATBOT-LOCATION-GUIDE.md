# Chatbot Widget Location & Visibility Guide

## 📍 Where is the Chat Widget?

The chatbot widget appears as a **floating button in the bottom-right corner** of every page:

### Location Details:
- **Position**: Fixed at bottom-right corner
- **Distance from edges**: 24px from bottom, 24px from right
- **Z-index**: 9999 (always on top)
- **Size**: 70px × 70px circular button

### Visual Appearance:
- 🤖 **Robot icon** in the center
- **Cyan/blue gradient background**
- **Glowing border** (cyan color)
- **Shadow effect** for depth

## 🎯 How to Use:

1. **Find the Button**: Look for the circular robot icon (🤖) in the bottom-right corner
2. **Click to Open**: Click the button to open the chat interface
3. **Start Chatting**: Type your question and press Enter or click Send
4. **Minimize**: Click the "−" button to minimize
5. **Close**: Click the "×" button to close

## 🔍 If You Can't See It:

### Check These:
1. **Browser Console**: Open DevTools (F12) and check for errors
2. **Z-index Conflicts**: Make sure no other element is covering it
3. **Viewport**: Scroll to bottom-right of page
4. **Client Module**: Verify `client-module.tsx` is loaded
5. **CSS**: Ensure styles are compiled

### Quick Test:
```javascript
// In browser console, run:
document.querySelector('.chatWidgetContainer')
// Should return the widget container element
```

## 📱 Mobile View:
- Button size: 60px × 60px
- Position: 16px from bottom and right
- Fully responsive and touch-friendly

## ✅ Status:
The chatbot widget is **always visible** on all pages when:
- ✅ Client module is loaded
- ✅ No JavaScript errors
- ✅ CSS is properly compiled
- ✅ Z-index is set correctly (9999)

## 🎨 Styling:
- **Background**: Dark blue gradient (#0f3460 to #16213e)
- **Border**: Cyan with glow effect
- **Icon**: Robot emoji (🤖) with subtle glow
- **Hover**: Slight scale up (1.05x) and brighter glow

---

**Note**: The widget is registered globally via `clientModules` in `docusaurus.config.ts` and renders on every page automatically.

