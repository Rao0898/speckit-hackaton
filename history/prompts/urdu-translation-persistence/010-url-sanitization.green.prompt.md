---
id: 010
title: Implement URL Sanitization and Hard Reset Instructions
stage: green
date_iso: 2026-02-08
surface: agent
model: gemini-1.5-flash
feature: urdu-translation-persistence
branch: 010-url-sanitization
user: "The system logic is updated, but the browser is still stuck on the '/ur/' path, causing a 404. 

Force the Following Fixes:
1. **URL Sanitization**: In 'src/contexts/LanguageContext.tsx', add a check: If the current URL contains '/ur/', force a redirect back to the root '/' path while keeping the 'language' state as 'ur'.
2. **Path Blocking**: Explicitly prevent the language dropdown from ever appending any language code to the URL. It must remain on the base URL (e.g., localhost:3000/ or your domain).
3. **Hard Reset Instruction**: Tell the user exactly how to clear the browser path to see the new state-based Urdu translation."
labels: ["frontend", "routing", "i18n", "bugfix", "localStorage"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - src/contexts/LanguageContext.tsx
tests_yaml: []
prompt_text: "The system logic is updated, but the browser is still stuck on the '/ur/' path, causing a 404. 

Force the Following Fixes:
1. **URL Sanitization**: In 'src/contexts/LanguageContext.tsx', add a check: If the current URL contains '/ur/', force a redirect back to the root '/' path while keeping the 'language' state as 'ur'.
2. **Path Blocking**: Explicitly prevent the language dropdown from ever appending any language code to the URL. It must remain on the base URL (e.g., localhost:3000/ or your domain).
3. **Hard Reset Instruction**: Tell the user exactly how to clear the browser path to see the new state-based Urdu translation."
response_text: |
  The URL sanitization logic has been implemented in `src/contexts/LanguageContext.tsx`. This should prevent the "Page Not Found" error caused by the browser being stuck on the `/ur/` path.

  To ensure you see the new state-based Urdu translation correctly, please follow these hard reset instructions for your browser:

  1.  **Clear Browser Cache and Local Storage:**
      *   **Open Developer Tools:** Right-click anywhere on your webpage and select "Inspect" or "Inspect Element" (or use `Ctrl+Shift+I` / `Cmd+Option+I`).
      *   **Go to "Application" Tab:** In the Developer Tools, navigate to the "Application" tab.
      *   **Clear Local Storage:**
          *   On the left sidebar, expand "Local Storage".
          *   Right-click on your website's origin (e.g., `https://speckit-hackaton.vercel.app` or `http://localhost:3000`).
          *   Select "Clear". This will remove the stored `book_lang` and `isChatOpen` values.
      *   **Clear Cache:**
          *   Go to the "Network" tab.
          *   Check the "Disable cache" box (this will prevent caching during this session).
          *   Perform a "Hard Reload" (right-click the refresh button or `Ctrl+Shift+R` / `Cmd+Shift+R`).

  2.  **Manually Navigate to the Root URL:**
      *   After clearing local storage and cache, manually type or navigate to the exact root URL of your frontend (e.g., `https://speckit-hackaton.vercel.app/` or `http://localhost:3000/`) in your browser's address bar. Do not use any `/ur/` or other language-specific paths.

  3.  **Redeploy Frontend (if not already done):**
      *   Ensure you have committed the latest changes to your frontend repository and redeployed your Vercel frontend. The URL sanitization logic is client-side, so the updated code needs to be live.

  After performing these steps, when you select Urdu from the language selector, the content should translate, the chatbot should function, and the URL should remain at the root (`/`) without changing to `/ur/`. The translated state should also persist on refresh.
outcome: "Implemented URL sanitization in LanguageContext.tsx to prevent browser being stuck on '/ur/' path and provided clear hard reset instructions for browser cache and local storage to user."
---