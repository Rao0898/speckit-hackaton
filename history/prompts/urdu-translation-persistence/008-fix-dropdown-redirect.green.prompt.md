---
id: 008
title: Fix Language Dropdown URL Redirect and Ensure UI Persistence
stage: green
date_iso: 2026-02-08
surface: agent
model: gemini-1.5-flash
feature: urdu-translation-persistence
branch: 008-fix-dropdown-redirect
user: "The user is facing a \"Page Not Found\" error because the language dropdown is triggering a URL redirect to '/ur/'. 

STRICT INSTRUCTIONS to fix the UI:
1. **Remove URL Redirects**: Modify the language dropdown in 'src/pages/index.tsx' or the Header component. It must NOT change the window location or add '/ur/' to the URL.
2. **State-Only Toggle**: Change the 'onChange' handler of the dropdown so it only calls 'setLanguage(\"ur\")' from the 'LanguageContext' and updates 'localStorage'. 
3. **Dropdown Sync**: Ensure the dropdown's 'value' is tied directly to the 'language' state from 'LanguageContext', so it correctly shows \"Urdu\" when selected, even if the user stays on the same page.
4. **Fix \"Stuck\" English**: The dropdown is showing \"English\" while the URL is '/ur/'. Fix this by ensuring the component re-renders based on the 'LanguageContext' value on initial load.

Do not use Docusaurus i18n routing. Use internal React state for language switching."
labels: ["frontend", "routing", "i18n", "persistence", "docusaurus", "bugfix"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - src/pages/index.tsx
tests_yaml: []
prompt_text: "The user is facing a \"Page Not Found\" error because the language dropdown is triggering a URL redirect to '/ur/'. 

STRICT INSTRUCTIONS to fix the UI:
1. **Remove URL Redirects**: Modify the language dropdown in 'src/pages/index.tsx' or the Header component. It must NOT change the window location or add '/ur/' to the URL.
2. **State-Only Toggle**: Change the 'onChange' handler of the dropdown so it only calls 'setLanguage(\"ur\")' from the 'LanguageContext' and updates 'localStorage'. 
3. **Dropdown Sync**: Ensure the dropdown's 'value' is tied directly to the 'language' state from 'LanguageContext', so it correctly shows \"Urdu\" when selected, even if the user stays on the same page.
4. **Fix \"Stuck\" English": The dropdown is showing \"English\" while the URL is '/ur/'. Fix this by ensuring the component re-renders based on the 'LanguageContext' value on initial load.

Do not use Docusaurus i18n routing. Use internal React state for language switching."
response_text: | 
  All the tasks to address the "Page Not Found" error due to unintended localized routing and to ensure proper UI persistence for language and chatbot visibility have been completed.

  Here's a summary of the final changes:

  *   **`src/pages/index.tsx`**:
      *   The language selector was reverted back to a `<select>` element.
      *   The `onChange` handler for the `<select>` element now *only* calls `setLanguage(event.target.value)` from the `LanguageContext`. This ensures state-only toggling and prevents any implicit URL redirects.
      *   The `value` of the `<select>` element is directly tied to the `language` state from `LanguageContext`, ensuring the dropdown accurately reflects the current language.
      *   Chatbot visibility persistence (using `isChatOpen` in `localStorage`) remains independent of the language state.
  *   **`src/contexts/LanguageContext.tsx`**: Confirmed to correctly handle language persistence by initializing the language state from `localStorage` and updating `localStorage` whenever the language changes.
  *   **`src/components/Chatbot/index.tsx`**: Confirmed to correctly send the `language` parameter to the backend with API calls, ensuring the backend receives the selected language for translation.

  To make these changes effective, the user needs to:
  1.  **Commit the changes to their frontend repository.**
  2.  **Redeploy their Vercel frontend.**
outcome: "Resolved URL redirect issue with language dropdown, ensuring state-only language switching and proper UI persistence for language and chatbot visibility."
---
