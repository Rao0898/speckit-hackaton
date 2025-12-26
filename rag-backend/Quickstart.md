# Quickstart: RAG Chatbot Backend for Physical AI & Humanoid Robotics Textbook

This guide provides a step-by-step setup for the RAG (Retrieval Augmented Generation) chatbot backend, designed for the Physical AI & Humanoid Robotics textbook.

## 1. Architecture Overview

This backend is built with the following architecture:

-   **Gemini:** Utilized exclusively as the conversational agent for understanding user questions and generating final answers. Gemini is **NOT** used for embeddings.
-   **Voyage AI:** Used exclusively for generating document embeddings. Recommended model: `voyage-2`.
-   **Qdrant Cloud:** Serves as the vector store for efficient similarity search of document embeddings. Configured via `QDRANT_URL` and `QDRANT_API_KEY` environment variables.
-   **Neon Postgres:** Used as the relational database for storing metadata associated with the document chunks. Configured via the `DATABASE_URL` environment variable.

## 2. Prerequisites

Before you begin, ensure you have the following installed and configured:

-   **Python 3.11+**
-   **Poetry** for dependency management.
-   **Google Gemini API Key:** For the conversational agent.
-   **Voyage AI API Key:** For generating embeddings.
-   **Qdrant Cloud Account:** For your vector database. Obtain your `QDRANT_URL` and `QDRANT_API_KEY`.
-   **Neon Postgres Database:** For your metadata store. Obtain your `DATABASE_URL`.

## 3. Step-by-step Instructions

Follow these steps to get your RAG backend up and running:

### Step 1: Navigate to the `rag-backend` directory

```bash
cd D:\hackaton-specify\humanoid-book\rag-backend
```

### Step 2: Install Python dependencies

Use Poetry to install all required dependencies.

```bash
poetry install
```

### Step 3: Create your `.env` file

Create a `.env` file in the `rag-backend` directory and populate it with your API keys and database connection strings. Replace the placeholder values with your actual credentials.

```env
GEMINI_API_KEY="YOUR_GEMINI_API_KEY"
VOYAGE_API_KEY="YOUR_VOYAGE_AI_API_KEY"
QDRANT_URL="YOUR_QDRANT_CLOUD_URL"
QDRANT_API_KEY="YOUR_QDRANT_CLOUD_API_KEY"
DATABASE_URL="YOUR_NEON_POSTGRES_DATABASE_URL"
```

### Step 4: Run the ingestion script

This script will read markdown files from the `docs` directory, chunk their content, generate embeddings using Voyage AI, store the vectors in Qdrant Cloud, and store associated metadata in Neon Postgres.

```bash
poetry run python -m src.scripts.ingest
```

**Note:** This command will create the necessary tables in your Neon Postgres database and the collection in Qdrant Cloud if they don't already exist. You might encounter a `pydantic_core._pydantic_core.ValidationError` if your environment variables are not correctly set in the `.env` file or if they are not loaded properly. Ensure all values are correctly specified.

### Step 5: Start the FastAPI server

Once ingestion is complete, you can start the FastAPI application server.

```bash
poetry run uvicorn src.main:app --reload
```

### Step 6: Test the API

Open your web browser and navigate to `http://127.0.0.1:8000/docs`. This will open the OpenAPI (Swagger UI) documentation, where you can explore and test the available API endpoints.

## 4. Important Warnings

-   **DO NOT use Gemini for embeddings:** Gemini is reserved for chat and reasoning.
-   **DO NOT use localhost Qdrant:** Ensure you are using your Qdrant Cloud instance.
-   **DO NOT use SQLite:** The backend is configured for Neon Postgres.
-   **DO NOT use relative imports:** All imports must be absolute (`from src.module import ...`).

## 5. Final Outcome

Upon successful completion of these steps, you will have a fully functional RAG backend that is:

-   **Hackathon-ready:** Quickly deployable for your projects.
-   **Cloud-deployable:** Designed for cloud environments.
-   **Production-grade RAG backend:** Robust and scalable for real-world applications.
