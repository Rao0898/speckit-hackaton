---
phr_id: "7"
title: "Fix Frontend-Backend API Mismatch"
stage: "refactor"
date: "2025-12-15"
surface: "agent"
model: "gemini-1.5-pro"
feature: "002-rag-backend"
branch: "002-rag-backend"
user: "default"
command: "Check the frontend and backend integration.\n\nThere is a mismatch between the frontend and backend:\n\n1. The frontend currently calls http://localhost:8000/rag-answer, but the backend provides /api/v1/query and /api/v1/query-selection.\n2. The frontend sends a request body with a 'context' field, but the backend expects a 'selection' field for selection-based queries.\n\nYour task: Fix only this mismatch so that the frontend calls the correct backend endpoints with the correct request body.\n\nDo not make any other changes to the frontend or backend code. Only fix this API mismatch."
labels: ["api-integration", "frontend", "backend"]
links:
  spec: "specs/002-rag-backend/spec.md"
  ticket: "null"
  adr: "null"
  pr: "null"
files_yaml:
  - "src/components/Chatbot/index.tsx"
tests_yaml: []
prompt_text: |
  Check the frontend and backend integration.

  There is a mismatch between the frontend and backend:

  1. The frontend currently calls http://localhost:8000/rag-answer, but the backend provides /api/v1/query and /api/v1/query-selection.
  2. The frontend sends a request body with a 'context' field, but the backend expects a 'selection' field for selection-based queries.

  Your task: Fix only this mismatch so that the frontend calls the correct backend endpoints with the correct request body.

  Do not make any other changes to the frontend or backend code. Only fix this API mismatch.
response_text: |
  The API mismatch between the frontend and backend has been resolved.

  - **Modified File**: `src/components/Chatbot/index.tsx`

  The frontend now dynamically calls the correct backend endpoints (`/api/v1/query` or `/api/v1/query-selection`) and uses the `selection` field in the request body when a text selection is present.
---