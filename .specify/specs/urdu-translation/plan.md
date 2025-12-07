# Implementation Plan: Urdu Translation

## Phase 1: Translation Button Component
1. Create TranslationButton React component
2. Add to all chapter pages via MDX component
3. Handle toggle state (English/Urdu)

## Phase 2: Translation API
1. Create FastAPI endpoint for translation
2. Use Gemini API to translate content
3. Preserve code blocks and markdown structure
4. Cache translations in database

## Phase 3: Content Display
1. Fetch translated content when enabled
2. Display Urdu version of chapter
3. Allow toggle back to English

## Technical Implementation

### Frontend
- TranslationButton component
- Content display logic
- Language toggle state

### Backend
- `/api/translate` endpoint
- Gemini API integration
- Markdown structure preservation
- Code block preservation

### Database
```sql
translations (
  id, chapter_path, language, original_content, 
  translated_content, created_at, updated_at
)
```

