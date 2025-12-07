# Feature Specification: Urdu Translation

**Feature Branch**: `005-urdu-translation`  
**Created**: 2025-01-07  
**Status**: Draft  
**Input**: User description: "Add Urdu translation button in chapters and implement translation functionality"

## User Scenarios & Testing _(mandatory)_

### User Story 1 - Chapter Translation to Urdu (Priority: P1)

A logged-in user reading a chapter in English wants to read it in Urdu. They click the translation button, and the chapter content is translated to Urdu while preserving the structure and code examples.

**Why this priority**: This is a core bonus feature that makes the textbook accessible to Urdu-speaking students.

**Independent Test**: Can be tested by clicking translation button and verifying content is translated correctly.

**Acceptance Scenarios**:

1. **Given** a logged-in user is on a chapter page, **When** they click translation button, **Then** the content is translated to Urdu
2. **Given** content is translated, **When** user views the page, **Then** code examples remain in original language
3. **Given** user clicks translation again, **When** they toggle it, **Then** content returns to English
4. **Given** a non-logged-in user, **When** they try to translate, **Then** they are prompted to sign up

---

### User Story 2 - Translation Button in Chapters (Priority: P1)

A user reading a chapter sees a translation button. They can click it to toggle between English and Urdu.

**Why this priority**: Users need a clear way to access translation features.

**Independent Test**: Can be tested by checking that the button appears on all chapter pages and functions correctly.

**Acceptance Scenarios**:

1. **Given** a user is on a chapter page, **When** they view the page, **Then** they see a translation button
2. **Given** a user clicks translation, **When** content is translated, **Then** the button shows active state
3. **Given** a user toggles translation off, **When** they view content, **Then** original English content is displayed

---

## Requirements _(mandatory)_

### Functional Requirements

- **FR-001**: System MUST display translation button at start of each chapter
- **FR-002**: System MUST translate chapter content to Urdu
- **FR-003**: System MUST preserve code examples in original language
- **FR-004**: System MUST preserve markdown structure during translation
- **FR-005**: System MUST allow users to toggle translation on/off
- **FR-006**: System MUST require authentication for translation
- **FR-007**: System MUST use AI (Gemini) for translation
- **FR-008**: System MUST cache translations for performance
- **FR-009**: System MUST handle translation errors gracefully

### Key Entities _(include if feature involves data)_

- **Translation**: Represents translated content. Attributes: chapter_id, language (ur/en), original_content, translated_content, user_id, created_at, updated_at
- **TranslationCache**: Represents cached translations. Attributes: content_hash, language, translated_content, created_at

## Success Criteria _(mandatory)_

### Measurable Outcomes

- **SC-001**: Translation button appears on all chapter pages (100%)
- **SC-002**: Translation is generated within 10 seconds
- **SC-003**: Translated content is accurate (verified manually)
- **SC-004**: Code examples remain untranslated (100%)
- **SC-005**: Users can toggle translation successfully (95% success rate)
- **SC-006**: Markdown structure is preserved in translation

## Technical Constraints

- **AI Service**: Google Gemini API (for translation)
- **Frontend**: React component in Docusaurus pages
- **Backend**: FastAPI endpoint for translation
- **Caching**: Store translations to reduce API calls
- **Database**: Neon Postgres (for storing translations)
- **Language**: Urdu (ur) - already configured in Docusaurus i18n

## Non-Goals

- Automatic translation (user must opt-in)
- Translation of code comments (preserve original)
- Multi-language support beyond English/Urdu

## Dependencies

- Authentication system (Better Auth)
- Gemini API access
- Docusaurus i18n configuration (already set up)
- Docusaurus page structure

