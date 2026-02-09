---
id: 006
title: Implement Urdu Translation and UI Persistence
stage: green
date_iso: 2026-02-08
surface: agent
model: gemini-1.5-flash
feature: urdu-translation-persistence
branch: 006-urdu-translation
user: "Act as a Full-Stack Developer and Technical Translator. Your task is to implement Urdu translation and UI persistence exactly as described below, without changing existing chatbot core logic.

1. **Translation Logic (Standard from Video)**:
   - **Quality**: Use 'Contextual Translation'. Instead of word-to-word, translate technical concepts into easy-to-understand academic Urdu.
   - **Source Content**: Access the textbook data from the Qdrant vector database via the `rag_service.py`.
   - **Terminology**: Keep English technical terms (e.g., "ROS 2", "Nodes", "Docker") in English script or brackets, but explain the context in Urdu.

2. **File Roles & Functionality**:
   - `rag_service.py`: This is the engine. It must fetch context from Qdrant and pass it to the LLM (Gemini) with a 'Translate to Urdu' instruction.
   - `query.py` (API Layer): Handles the incoming user request. It must support a `language` parameter or default to the system's sticky language.
   - `index.tsx` / `Chatbot.tsx` (Frontend): 
     - Must implement 'localStorage' to save the user's language choice.
     - On page load, check `localStorage.getItem('book_lang')`. If it is 'ur', keep the book and chatbot in Urdu mode.
     - Chatbot popup must check `localStorage.getItem('isChatOpen')` to stay open after refresh.

3. **Strict UI Behavior**:
   - **Persistence**: Once Urdu is selected, it is the 'Master State'. Refreshing the browser MUST NOT trigger a reset to English.
   - **Chatbot Visibility**: The chatbot is a sticky element. If it was open before refresh, it must be open after refresh.

4. **Response Protocol**:
   - For greetings (Hi/Hello/Assalam-o-Alaikum): Reply in Roman Urdu or Urdu script: "Walaikum Assalam! Main aapki textbook samajhne mein kaise madad kar sakta hoon?".
"
labels: ["full-stack", "translation", "i18n", "persistence", "frontend", "backend", "chatbot"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - rag-backend/src/models/query.py
  - rag-backend/src/api/query.py
  - rag-backend/src/services/rag_service.py
  - src/contexts/LanguageContext.tsx
  - src/pages/index.tsx
  - src/components/Chatbot/index.tsx
tests_yaml: []
prompt_text: "Act as a Full-Stack Developer and Technical Translator. Your task is to implement Urdu translation and UI persistence exactly as described below, without changing existing chatbot core logic.

1. **Translation Logic (Standard from Video)**:
   - **Quality**: Use 'Contextual Translation'. Instead of word-to-word, translate technical concepts into easy-to-understand academic Urdu.
   - **Source Content**: Access the textbook data from the Qdrant vector database via the `rag_service.py`.
   - **Terminology**: Keep English technical terms (e.g., "ROS 2", "Nodes", "Docker") in English script or brackets, but explain the context in Urdu.

2. **File Roles & Functionality**:
   - `rag_service.py`: This is the engine. It must fetch context from Qdrant and pass it to the LLM (Gemini) with a 'Translate to Urdu' instruction.
   - `query.py` (API Layer): Handles the incoming user request. It must support a `language` parameter or default to the system's sticky language.
   - `index.tsx` / `Chatbot.tsx` (Frontend): 
     - Must implement 'localStorage' to save the user's language choice.
     - On page load, check `localStorage.getItem('book_lang')`. If it is 'ur', keep the book and chatbot in Urdu mode.
     - Chatbot popup must check `localStorage.getItem('isChatOpen')` to stay open after refresh.

3. **Strict UI Behavior**:
   - **Persistence**: Once Urdu is selected, it is the 'Master State'. Refreshing the browser MUST NOT trigger a reset to English.
   - **Chatbot Visibility**: The chatbot is a sticky element. If it was open before refresh, it must be open after refresh.

4. **Response Protocol**:
   - For greetings (Hi/Hello/Assalam-o-Alaikum): Reply in Roman Urdu or Urdu script: "Walaikum Assalam! Main aapki textbook samajhne mein kaise madad kar sakta hoon?".
"
response_text: | All the planned tasks for implementing Urdu translation and UI persistence are now completed.

Here's a summary of the changes made:

**Backend:**
  *   `rag-backend/src/models/query.py`: `QueryRequest` model updated to include an optional `language` field (defaulting to "en").
  *   `rag-backend/src/api/query.py`: The `query_general` endpoint modified to accept and pass the `language` parameter to `rag_service.query_general`.
  *   `rag-backend/src/services/rag_service.py`:
      *   `query_general` method signature updated to accept `language`.
      *   Logic added to conditionally modify the prompt sent to the Gemini model to include "Translate to Urdu, keeping technical terms in English script" if `language` is 'ur'.
      *   The predefined greeting response updated to be in Roman Urdu for Urdu language requests.

**Frontend:**
  *   `src/contexts/LanguageContext.tsx`: A new context created to manage and persist the selected language using `localStorage`.
  *   `src/pages/index.tsx`:
      *   Integrated `LanguageProvider` to make language state available globally.
      *   A language selector (dropdown) added to `HomepageHeader` to allow users to switch between English and Urdu.
      *   Chatbot visibility persistence implemented using `localStorage.getItem('isChatOpen')` to control the rendering of the `Chatbot` component.
      *   A toggle button added to open/close the chatbot, with its state persisted in `localStorage`.
  *   `src/components/Chatbot/index.tsx`:
      *   Modified to consume the `LanguageContext` to get the current `language`.
      *   `mockFetch` implementation removed and replaced with actual `fetch` calls, including the `language` parameter in API requests.
      *   The `BACKEND_URL` from environment variables is now used for API endpoints.

To ensure all these changes are reflected, the user needs to:
  1.  Commit the changes to their repositories.
  2.  Redeploy their Render backend.
  3.  Redeploy their Vercel frontend.
outcome: "Implemented Urdu translation and UI persistence features across backend and frontend, including language selection, persistence, chatbot visibility persistence, and API integration. Provided redeployment instructions."
---