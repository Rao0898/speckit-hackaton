---
id: 007
title: Fix Urdu Translation Routing and Implement UI Persistence
stage: green
date_iso: 2026-02-08
surface: agent
model: gemini-1.5-flash
feature: urdu-translation-persistence
branch: 007-fix-urdu-routing
user: "The Urdu translation implementation is causing a \"422 Page Not Found\" because the URL is changing to '/ur/' which does not exist in the routing setup.\n\nFix the following:\n1. **Routing Fix**: Modify 'src/contexts/LanguageContext.tsx' and the language selector in 'src/pages/index.tsx' so that switching to Urdu ONLY updates the internal state and 'localStorage', WITHOUT changing the browser URL path to '/ur/'.\n2. **Persistence Check**: Ensure that even if the URL remains 'localhost:3000/', the 'LanguageContext' correctly triggers the 'ur' parameter in API calls to the backend based on the 'localStorage' value.\n3. **Chatbot Cleanup**: Ensure the chatbot remains open on the main page regardless of the language state, using the 'isChatOpen' localStorage key you already implemented.\n\nDo not use localized routing; use state-based translation only."
labels: ["frontend", "routing", "i18n", "persistence", "docusaurus"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - src/pages/index.tsx
tests_yaml: []
prompt_text: "The Urdu translation implementation is causing a \"422 Page Not Found\" because the URL is changing to '/ur/' which does not exist in the routing setup.\n\nFix the following:\n1. **Routing Fix**: Modify 'src/contexts/LanguageContext.tsx' and the language selector in 'src/pages/index.tsx' so that switching to Urdu ONLY updates the internal state and 'localStorage', WITHOUT changing the browser URL path to '/ur/'.\n2. **Persistence Check**: Ensure that even if the URL remains 'localhost:3000/', the 'LanguageContext' correctly triggers the 'ur' parameter in API calls to the backend based on the 'localStorage' value.\n3. **Chatbot Cleanup**: Ensure the chatbot remains open on the main page regardless of the language state, using the 'isChatOpen' localStorage key you already implemented.\n\nDo not use localized routing; use state-based translation only."
response_text: |
  All the tasks related to fixing the "422 Page Not Found" error and implementing persistence for language and chatbot visibility have been completed.

  Here's a summary of the changes:

  *   **`src/pages/index.tsx`**:
      *   The language selector was changed from a `select` element to two buttons (English/Urdu) to prevent Docusaurus from implicitly triggering localized routing.
      *   The language selection now *only* updates the internal state and `localStorage` without changing the browser URL.
      *   Chatbot visibility persistence is ensured using `isChatOpen` in `localStorage`, controlling the rendering of the `Chatbot` component and independent of the language state.
  *   **`src/contexts/LanguageContext.tsx`**: Verified to correctly handle language persistence by reading from `localStorage` on initialization and writing to `localStorage` on language change.
  *   **`src/components/Chatbot/index.tsx`**: Confirmed to correctly send the `language` parameter to the backend via API calls.

  To make these changes effective, the user needs to:
  1.  **Commit the changes to their frontend repository.**
  2.  **Redeploy their Vercel frontend.**
outcome: "Fixed Urdu translation routing issue by implementing state-based translation without URL changes, and ensured language/chatbot visibility persistence."
---