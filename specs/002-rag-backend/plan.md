# Implementation Plan: RAG Chatbot Backend

**Branch**: `002-rag-backend` | **Date**: 2025-12-15 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `specs/002-rag-backend/spec.md`

## Summary

This plan outlines the technical design for a Retrieval-Augmented Generation (RAG) chatbot backend. The system will be built with Python and FastAPI, using Qdrant for vector search and Neon Postgres for metadata storage. It will provide API endpoints for general and selection-based questioning, powered by an OpenAI-based agentic framework.

## Technical Context

**Language/Version**: Python 3.11
**Primary Dependencies**: FastAPI, OpenAI SDK, Qdrant client, psycopg2-binary
**Storage**: Neon Serverless Postgres (metadata), Qdrant Cloud (vectors)
**Testing**: pytest
**Target Platform**: Linux Server / containerized environment
**Project Type**: Backend service
**Performance Goals**: 95% of API responses delivered in under 3 seconds.
**Constraints**: Backend code located in `/rag-backend`.
**Scale/Scope**: Designed to support the content of the "Physical AI & Humanoid Robotics" textbook. User scale is not a primary design constraint for the hackathon.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- [x] **Docusaurus + GitHub Pages**: The backend is decoupled and will be compatible with the existing frontend.
- [x] **Full RAG Chatbot**: The plan includes all required components: FastAPI, Qdrant, Neon, and an OpenAI agent framework.
- [x] **Text-Selection Q&A**: The API design includes a dedicated endpoint for this functionality.
- [x] **Reusable Intelligence**: The choice of the OpenAI agent framework provides a foundation for reusable skills, though no custom "Claude Code Subagents" are in scope for this backend-only feature.

**Result**: All gates passed. The plan is compliant with the project constitution.

## Project Structure

### Documentation (this feature)

```text
specs/002-rag-backend/
├── plan.md              # This file
├── research.md          # Technology selection research
├── data-model.md        # Database schema for content chunks
├── quickstart.md        # Setup and running instructions
├── contracts/           # API contract definitions
│   └── openapi.yml
└── tasks.md             # To be created by /sp.tasks
```

### Source Code (repository root)

The backend code will be self-contained within the `/rag-backend` directory as mandated by the constitution.

```text
rag-backend/
├── src/
│   ├── api/             # FastAPI endpoints
│   ├── core/            # Core logic, configuration
│   ├── models/          # Pydantic models for API and data
│   ├── services/        # Business logic for RAG pipeline
│   └── scripts/         # Ingestion and utility scripts
└── tests/
    ├── integration/
    └── unit/
```

**Structure Decision**: A self-contained backend application structure is chosen to isolate the service from the Docusaurus frontend, as required.

## Complexity Tracking

No violations of the constitution were identified. This section is not needed.