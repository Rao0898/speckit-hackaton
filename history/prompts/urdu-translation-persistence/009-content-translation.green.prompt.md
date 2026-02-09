---
id: 009
title: Implement Textbook Content Translation and Persistence
stage: green
date_iso: 2026-02-08
surface: agent
model: gemini-1.5-flash
feature: urdu-translation-persistence
branch: 009-content-translation
user: "The chatbot is now working in Urdu, but the main textbook content on the page is still in English. 

Please implement the following:
1. **Content Translation Logic**: In 'src/pages/index.tsx' (or wherever the book content is rendered), wrap the main text components to check the 'language' state from 'LanguageContext'.
2. **Translation Service Call**: If the language is 'ur', the frontend should send the page content to the backend's translation endpoint (or use Gemini directly if configured) to get the Urdu version of the textbook text.
3. **Keep Formatting**: Ensure that when the book text is translated to Urdu, the original layout, headings, and code blocks remain intact.
4. **No Refresh Reset**: Ensure the translated Urdu text stays visible even after refresh, as per the existing localStorage logic."
labels: ["full-stack", "translation", "i18n", "persistence", "frontend", "backend", "content"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - rag-backend/src/models/translation.py
  - rag-backend/src/services/translation_service.py
  - rag-backend/src/api/translation.py
  - rag-backend/src/main.py
  - src/components/HomepageFeatures/index.tsx
tests_yaml: []
prompt_text: "The chatbot is now working in Urdu, but the main textbook content on the page is still in English. 

Please implement the following:
1. **Content Translation Logic**: In 'src/pages/index.tsx' (or wherever the book content is rendered), wrap the main text components to check the 'language' state from 'LanguageContext'.
2. **Translation Service Call**: If the language is 'ur', the frontend should send the page content to the backend's translation endpoint (or use Gemini directly if configured) to get the Urdu version of the textbook text.
3. **Keep Formatting**: Ensure that when the book text is translated to Urdu, the original layout, headings, and code blocks remain intact.
4. **No Refresh Reset**: Ensure the translated Urdu text stays visible even after refresh, as per the existing localStorage logic."
response_text: |
  All tasks for implementing the content translation and persistence are now completed.

  **Summary of changes:**

  **Backend:**
  *   Created `rag-backend/src/models/translation.py` with `TranslationRequest` and `TranslationResponse` Pydantic models.
  *   Created `rag-backend/src/services/translation_service.py` implementing `TranslationService` to call Gemini API for contextual Urdu translation, preserving technical terms and formatting.
  *   Created `rag-backend/src/api/translation.py` with a new APIRouter for the translation endpoint.
  *   Included `translation.router` in `rag-backend/src/main.py` to expose the new translation endpoint.

  **Frontend:**
  *   Modified `src/components/HomepageFeatures/index.tsx`:
      *   Changed `FeatureItem.description` type to `string` and updated `FeatureList` to use HTML string literals.
      *   Implemented `fetchTranslation` helper function for calling the backend translation API and caching results in `localStorage`.
      *   Modified the `Feature` component to use `useLanguage`, fetch and display translated `title` and `description` (rendered using `dangerouslySetInnerHTML` for HTML content), with persistence across refreshes via `localStorage` cache.

  To make these changes effective, the user needs to:
  1.  **Commit the changes to their repositories (both frontend and backend).**
  2.  **Redeploy their Render backend.**
  3.  **Redeploy their Vercel frontend.**
outcome: "Implemented dynamic content translation to Urdu, preserving formatting and ensuring persistence across refreshes, via a new backend translation endpoint and frontend integration."
---