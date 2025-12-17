# RAG Chatbot Backend

This directory contains the backend service for the "Physical AI & Humanoid Robotics" textbook RAG (Retrieval-Augmented Generation) chatbot. It provides API endpoints to answer general questions about the textbook content and specific questions based on user-selected text.

## Features

-   **FastAPI Backend**: Built with FastAPI for high performance and ease of use.
-   **Text Ingestion**: Processes Markdown content from the main Docusaurus `docs` directory.
-   **Content Chunking & Embedding**: Chunks text and generates vector embeddings using OpenAI's embedding models.
-   **Vector Search**: Utilizes Qdrant Cloud for efficient semantic search of content chunks.
-   **Metadata Storage**: Stores content chunk metadata in Neon Serverless Postgres.
-   **OpenAI Agent Integration**: Leverages OpenAI's chat completion for generating answers.
-   **General Q&A**: Answers questions based on the entire textbook content.
-   **Selection-Based Q&A**: Answers questions strictly based on a user-provided text selection.

## Project Structure

```
rag-backend/
├── src/
│   ├── api/             # FastAPI endpoints (e.g., query.py)
│   ├── core/            # Core utilities (config, database, qdrant, logger, exceptions)
│   ├── models/          # Pydantic models for API, SQLAlchemy models for DB
│   ├── services/        # Business logic (RAGService, ContentChunkDAO)
│   └── scripts/         # Utility scripts (e.g., ingest.py)
├── tests/               # Unit and integration tests
├── .env.example         # Example environment variables file
├── pyproject.toml       # Poetry project configuration
└── README.md            # This file
```

## Setup

### Prerequisites

-   Python 3.11 or later
-   Poetry for dependency management
-   Access to a Neon account (for Postgres)
-   Access to a Qdrant Cloud account (free tier is sufficient)
-   An OpenAI API key

### Installation

1.  **Clone the repository**:
    ```bash
    git clone https://github.com/your-org/humanoid-book.git
    cd humanoid-book/rag-backend
    ```

2.  **Install dependencies**:
    ```bash
    poetry install
    ```

3.  **Configure Environment Variables**:
    Create a `.env` file in the `rag-backend` directory based on `.env.example`.

    ```env
    # .env

    # OpenAI
    OPENAI_API_KEY="sk-your-openai-api-key"

    # Qdrant
    QDRANT_URL="https://your-qdrant-url.cloud"
    QDRANT_API_KEY="your-qdrant-api-key" # Use an API key for cloud access

    # Neon (Postgres)
    DATABASE_URL="postgresql://user:password@host:port/database"
    ```

## Usage

### 1. Initialize Database and Ingest Content

First, you need to create the database schema and ingest the textbook content.

```bash
poetry run python -m src.scripts.ingest
```
This script will:
-   Create necessary tables in your Neon Postgres database.
-   Scan Markdown files in the parent `docs` directory.
-   Chunk content, generate embeddings, and store them in Qdrant and Postgres.

### 2. Run the FastAPI Service

```bash
poetry run uvicorn src.main:app --reload
```
The API will be available at `http://127.0.0.1:8000`.

### 3. Access API Documentation

Once the server is running, open your browser to `http://127.0.0.1:8000/docs` to access the interactive Swagger UI.

## API Endpoints

-   `POST /api/v1/query`: For general questions about the textbook.
-   `POST /api/v1/query-selection`: For questions based on a provided text selection.

## Contributing

Please refer to the main repository's `CONTRIBUTING.md` for guidelines.
