# Tasks: Physical AI & Humanoid Robotics Textbook

**Input**: Design documents from `specs/001-spec-physical-ai-textbook/`

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure for both the Docusaurus site and the RAG backend.

- [x] T001 Initialize Docusaurus project in `my-book/` using `npx create-docusaurus@latest my-book classic` (already run, but keeping for traceability)
- [x] T002 [P] Initialize FastAPI project in `rag-backend/` with a `main.py` and `requirements.txt`.
- [x] T003 [P] Configure a `.env` file for the RAG backend with placeholder variables for Qdrant, Neon, and OpenAI keys in `rag-backend/.env`.
- [x] T004 [P] Create the basic directory structure for content: `modules/`, `chapters/`, `labs/` at the root.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure for the RAG backend.

- [x] T005 Implement the data models from `data-model.md` in `rag-backend/src/models.py`.
- [x] T006 Create a service in `rag-backend/src/database.py` to connect to Qdrant and Neon Postgres.
- [x] T007 [P] Implement a script in `rag-backend/src/ingest.py` to read markdown files, chunk them, generate embeddings, and store them in the databases.
- [x] T008 [P] Implement the `/search` endpoint from `contracts/api.yaml` in `rag-backend/src/main.py`.

---

## Phase 3: User Story 1 - Foundational Learning (Priority: P1) 🎯 MVP

**Goal**: Create the initial set of content for the textbook, focusing on ROS 2 fundamentals.

**Independent Test**: A user can navigate to the "ROS 2" module on the Docusaurus site and read the introductory chapters and labs. The content should be rendered correctly.

### Implementation for User Story 1

- [x] T009 [US1] Create the directory structure for Module 1 in `my-book/docs/module1-ros2/`.
- [x] T010 [P] [US1] Write the content for the first chapter "Introduction to ROS 2" in `my-book/docs/module1-ros2/01-intro.md`.
- [x] T011 [P] [US1] Write the content for the chapter "ROS 2 Nodes and Topics" in `my-book/docs/module1-ros2/02-nodes-topics.md`.
- [x] T012 [P] [US1] Write the first lab "Creating a Simple Publisher/Subscriber" in `my-book/docs/module1-ros2/lab1-pub-sub.md`.
- [x] T013 [US1] Update the Docusaurus sidebar in `my-book/sidebars.ts` to include the new Module 1 content.
- [x] T014 [US1] Run the ingestion script (`rag-backend/src/ingest.py`) to process the new Module 1 content.

---

## Phase 4: User Story 2 - Simulation and Interaction (Priority: P2)

**Goal**: Add content for simulating robots in Gazebo and Unity.

**Independent Test**: A user can access the "Simulation" module and follow the labs to launch a simulated robot.

### Implementation for User Story 2

- [x] T015 [US2] Create the directory structure for Module 2 in `my-book/docs/module2-simulation/`.
- [x] T016 [P] [US2] Write the content for the chapter "Introduction to Gazebo" in `my-book/docs/module2-simulation/01-gazebo-intro.md`.
- [x] T017 [P] [US2] Write the lab "Simulating a URDF Model" in `my-book/docs/module2-simulation/lab1-gazebo-urdf.md`.
- [x] T018 [US2] Update the Docusaurus sidebar in `my-book/sidebars.ts` to add Module 2.
- [x] T019 [US2] Run the ingestion script (`rag-backend/src/ingest.py`) to process the new Module 2 content.

---

## Phase 5: User Story 3 - Advanced AI Integration (Priority: P3)

**Goal**: Integrate the RAG chatbot into the Docusaurus site and add advanced content.

**Independent Test**: A user can ask a question in the chatbot on the Docusaurus site and receive an answer based on the textbook's content.

### Implementation for User Story 3

- [x] T020 [P] [US3] Implement the `http://127.0.0.1:8000/api/v1/query` endpoint from `contracts/api.yaml` in `rag-backend/src/main.py`.
- [x] T021 [US3] Create a new React component for the chatbot UI in `my-book/src/components/Chatbot/`.
- [x] T022 [US3] Add the `Chatbot` component to the Docusaurus layout in `my-book/src/theme/Layout/index.tsx`.
- [x] T023 [P] [US3] Write content for Module 3 "NVIDIA Isaac" in `my-book/docs/module3-nvidia-isaac/`.
- [x] T024 [P] [US3] Write content for Module 4 "Voice-to-Action" in `my-book/docs/module4-vla/`.
- [x] T025 [US3] Run the ingestion script (`rag-backend/src/ingest.py`) to process all new content.

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Final improvements and deployment.

- [ ] T026 [P] Review all content for clarity, consistency, and correctness.
- [ ] T027 [P] Add diagrams and images to all modules.
- [ ] T028 Configure and test the deployment to GitHub Pages.
- [ ] T029 Deploy the RAG backend to a cloud service (e.g., Vercel, Railway).
- [ ] T030 Final validation of the `quickstart.md` guide.

---

## Dependencies & Execution Order

- **Setup (Phase 1)** and **Foundational (Phase 2)** must be completed before any user story work begins.
- **User Stories (Phases 3-5)** can be worked on sequentially or in parallel.
  - US1, US2, and US3 are largely independent from a content-creation perspective.
  - US3 (chatbot integration) depends on the backend from Phase 2 being complete.
- **Polish (Phase 6)** is the final phase.

## Implementation Strategy

### MVP First (User Story 1)

1. Complete Phase 1 & 2.
2. Complete all tasks in Phase 3 (User Story 1).
3. **STOP and VALIDATE**: The Docusaurus site should be live with the initial ROS 2 content.
4. This represents the first deliverable version of the textbook.
