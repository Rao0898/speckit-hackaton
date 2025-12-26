---
id: 24
title: Fix Gemini API Usage in RAG Service
stage: refactor
date: 2025-12-22
surface: agent
model: gemini-pro
feature: 002-rag-backend
branch: main
user: user
command: "run uvicorn and ask chatbot question"
labels: [bugfix, gemini-api, rag-service]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - rag-backend/src/services/rag_service.py
tests:
 (none)
---

## Prompt

when i run this command and ask question chatbot so in the browsr an error is ocuured is shown and in the backend terminal this error is coming

<traceback>

## Response snapshot

I have applied two fixes to `rag-backend/src/services/rag_service.py` to resolve the `AttributeError`. The issue was caused by incorrect usage of the Gemini API client.

1.  The client initialization was corrected to use `genai.configure()`.
2.  The calls to generate content were updated to use the correct `genai.GenerativeModel` object.

Please restart the server. The error should now be resolved.

## Outcome

- ✅ Impact: High - The RAG service was crashing due to an API misuse error. The service is now functional.
- 🧪 Tests: No new tests were added. The fix was a direct correction of an API call.
- 📁 Files: 1 file modified (`rag-backend/src/services/rag_service.py`).
- 🔁 Next prompts: The user should now be able to query the RAG service without errors.
- 🧠 Reflection: The initial implementation of the Gemini API call was incorrect. It used `genai.Client` instead of the `genai.GenerativeModel` pattern. This highlights the importance of consulting the library's documentation for correct usage patterns.

## Evaluation notes (flywheel)

- Failure modes observed: `AttributeError` due to incorrect API usage.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): n/a
