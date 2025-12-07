# Language Configuration - Urdu as Default

## ✅ Fixed: Urdu as Default Language

### Changes Made:
- Changed `defaultLocale` from `"en"` to `"ur"` in `docusaurus.config.ts`
- Reordered locales array: `["ur", "en"]` (Urdu first)

### Configuration:
```typescript
i18n: {
  defaultLocale: "ur", // Urdu as default language
  locales: ["ur", "en"], // Urdu first, then English
}
```

## 🌐 How It Works:

1. **Default Language**: Site loads in Urdu by default
2. **Language Switcher**: Users can switch to English via the language dropdown in navbar
3. **URL Structure**: 
   - Urdu (default): `/` or `/ur/`
   - English: `/en/`

## 📝 Next Steps for Full Urdu Support:

To fully support Urdu, you need to:

1. **Create Urdu Translations**:
   - Add `i18n/ur/` directory
   - Translate all markdown files
   - Translate UI strings

2. **Example Structure**:
   ```
   i18n/
     ur/
       docusaurus-theme-classic/
         navbar.json
         footer.json
       code.json
     en/
       ... (English translations)
   ```

3. **Translate Content**:
   - All docs in `docs/` need Urdu versions
   - Homepage content
   - Navigation labels
   - Footer text

## 🔧 Current Status:

- ✅ Default locale set to Urdu
- ✅ Language switcher available
- ⚠️ Content translations needed (if you want full Urdu content)

## 📚 Resources:

- [Docusaurus i18n Documentation](https://docusaurus.io/docs/i18n/introduction)
- [Translation Guide](https://docusaurus.io/docs/i18n/tutorial)

---

**Note**: The site will now default to Urdu locale, but you'll need to add Urdu translations for the content to appear in Urdu.

