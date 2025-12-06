# Quickstart Guide

**Date**: 2025-12-06
**Context**: Setup and launch instructions for the "Physical AI & Humanoid Robotics" project.

This guide provides the essential steps to get the Docusaurus textbook site and the RAG backend running locally.

## 1. Prerequisites

- **Node.js**: v18.x or later
- **Python**: 3.10 or later
- **Git**: Latest version
- **Docker**: For running Qdrant and Postgres locally (optional, if not using cloud services).

## 2. Docusaurus Textbook Site Setup

This will run the frontend of the textbook.

```bash
# 1. Clone the repository
git clone [repository-url]
cd humanoid-book

# 2. Install dependencies
npm install

# 3. Start the development server
npm start
```

The site will be available at `http://localhost:3000`.

## 3. RAG Backend Setup

This will run the FastAPI server that powers the chatbot.

### Using Cloud Services (Recommended for Hackathon)

1.  **Set up environment variables**:
    - Create a `.env` file in the `rag-backend/` directory.
    - Add the following variables with your credentials from Qdrant Cloud and Neon Postgres:
      ```env
      QDRANT_URL=...
      QDRANT_API_KEY=...
      POSTGRES_URL=...
      OPENAI_API_KEY=...
      ```

2.  **Install Python dependencies**:
    ```bash
    cd rag-backend
    pip install -r requirements.txt
    ```

3.  **Run the ingestion script** (to populate the vector store):
    ```bash
    python src/ingest.py
    ```

4.  **Start the FastAPI server**:
    ```bash
    uvicorn src.api.main:app --reload
    ```

The API will be available at `http://localhost:8000`, with documentation at `http://localhost:8000/docs`.

### Using Local Docker Containers

1.  **Launch local databases**:
    ```bash
    cd rag-backend
    docker-compose up -d
    ```
    This will start Qdrant and Postgres containers.

2.  **Set up environment variables**:
    - Create a `.env` file in `rag-backend/` pointing to the local containers:
      ```env
      QDRANT_URL=http://localhost:6333
      POSTGRES_URL=postgresql://user:password@localhost:5432/db
      OPENAI_API_KEY=...
      ```

3.  **Follow steps 2-4 from the "Using Cloud Services" section** to install dependencies, run ingestion, and start the server.
