# Tasks: RAG Chatbot Backend

**Input**: Design documents from `specs/002-rag-backend/`
**Prerequisites**: `plan.md`, `spec.md`, `data-model.md`, `contracts/openapi.yml`

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure for the `rag-backend`.

- [x] T001 Create the backend project directory structure in `rag-backend/`
- [x] T002 Initialize a new Python project with Poetry in `rag-backend/pyproject.toml`
- [x] T003 Add main dependencies (fastapi, uvicorn, openai, qdrant-client, psycopg2-binary, pydantic, python-dotenv) to `rag-backend/pyproject.toml`
- [x] T004 [P] Create an initial `.gitignore` file in `rag-backend/`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented.

- [x] T005 Create the main FastAPI application file in `rag-backend/src/main.py`
- [x] T006 [P] Implement configuration management to load environment variables in `rag-backend/src/core/config.py`
- [x] T007 [P] Implement the Qdrant client setup in `rag-backend/src/core/qdrant.py`
- [x] T008 [P] Implement the Postgres database connection logic in `rag-backend/src/core/database.py`

**Checkpoint**: Foundation ready - user story implementation can now begin.

---

## Phase 3: User Story 1 - General Textbook Q&A (Priority: P1) 🎯 MVP

**Goal**: Allow users to ask general questions about the textbook and receive answers grounded in its content.

**Independent Test**: Send a request to the `/query` endpoint and verify the response is accurate and contains citations.

### Implementation for User Story 1

- [x] T009 [P] [US1] Define API request/response models in `rag-backend/src/models/query.py` based on `contracts/openapi.yml`.
- [x] T010 [P] [US1] Define the data access object for `ContentChunk` in `rag-backend/src/services/content_chunk_dao.py`.
- [x] T011 [US1] Implement the content ingestion script logic in `rag-backend/src/scripts/ingest.py`.
- [x] T012 [US1] Implement the core RAG service for general queries in `rag-backend/src/services/rag_service.py`.
- [x] T013 [US1] Implement the `/query` API endpoint in `rag-backend/src/api/query.py`.
- [x] T014 [US1] Add the query router to the main FastAPI app in `rag-backend/src/main.py`.

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently.

---

## Phase 4: User Story 2 - Selection-Based Q&A (Priority: P2)

**Goal**: Allow users to ask questions about a specific passage of text.

**Independent Test**: Send a request to the `/query-selection` endpoint and verify the answer is derived only from the provided text.

### Implementation for User Story 2

- [x] T015 [P] [US2] Extend the API models in `rag-backend/src/models/query.py` to include the `QuerySelectionRequest`.
- [x] T016 [US2] Extend the RAG service in `rag-backend/src/services/rag_service.py` with a method for selection-based queries.
- [x] T017 [US2] Implement the `/query-selection` API endpoint in `rag-backend/src/api/query.py`.

**Checkpoint**: User Stories 1 and 2 should now both work independently.

---

## Phase 5: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories.

- [x] T018 [P] Add structured logging to all services and API endpoints.
- [x] T019 [P] Implement standardized exception handling middleware in `rag-backend/src/core/exceptions.py`.
- [x] T020 [P] Write the `README.md` for the `rag-backend` project.
- [ ] T021 Validate the setup and execution by following `specs/002-rag-backend/quickstart.md`.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: Can start immediately.
- **Foundational (Phase 2)**: Depends on Setup completion.
- **User Stories (Phase 3 & 4)**: Depend on Foundational phase completion.
- **Polish (Phase 5)**: Depends on all desired user stories being complete.

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2).
- **User Story 2 (P2)**: Can start after Foundational (Phase 2). It is independent of US1.

### Parallel Opportunities

- Within Phase 2, `config`, `qdrant`, and `database` setup can be done in parallel.
- Once Phase 2 is complete, User Story 1 and User Story 2 can be developed in parallel by different developers.
- Within each user story, tasks marked with `[P]` can be worked on in parallel.

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1.  Complete Phase 1: Setup
2.  Complete Phase 2: Foundational
3.  Complete Phase 3: User Story 1
4.  **STOP and VALIDATE**: Test User Story 1 independently. The backend is now minimally viable.

### Incremental Delivery

1.  Complete Setup + Foundational.
2.  Add User Story 1 → Test independently → MVP is ready.
3.  Add User Story 2 → Test independently → Full feature set is ready.
4.  Complete Polish phase.
