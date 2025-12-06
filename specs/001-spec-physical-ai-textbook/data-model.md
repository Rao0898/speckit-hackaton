# Data Model for RAG Backend

**Date**: 2025-12-06
**Context**: Defines the data structures for the RAG chatbot backend.

## 1. Core Entities

This data model focuses on the entities required for the Retrieval-Augmented Generation (RAG) system.

### DocumentChunk

This is the primary entity that will be stored and indexed for retrieval. The textbook content will be broken down into these chunks.

- **`chunk_id`** (UUID, Primary Key): Unique identifier for the chunk.
- **`document_id`** (String): Identifier for the source document (e.g., the path to the markdown file).
- **`content`** (Text): The raw text content of the chunk.
- **`metadata`** (JSONB): A flexible field to store contextual information.
  - `chapter`: The chapter the chunk belongs to.
  - `section`: The section within the chapter.
  - `page_number`: (Optional) The page number if applicable.
  - `tags`: An array of keywords or tags (e.g., "ROS 2", "URDF", "Simulation").
- **`created_at`** (Timestamp): When the chunk was created.

### VectorEmbedding

This entity represents the vector embedding of a `DocumentChunk`, stored in Qdrant.

- **`vector_id`** (UUID): The identifier for the vector, which will match the `chunk_id` of the corresponding `DocumentChunk`.
- **`vector`** (Array of Floats): The high-dimensional vector representation of the `content`.
- **`payload`** (JSON): A copy of the `metadata` from the `DocumentChunk` to allow for filtering directly within Qdrant.

## 2. Relationships

- **`DocumentChunk` to `VectorEmbedding`**: A one-to-one relationship. Each `DocumentChunk` in the Postgres database will have a corresponding `VectorEmbedding` in the Qdrant vector store, linked by a shared ID (`chunk_id` == `vector_id`).

## 3. Data Flow

1.  **Ingestion**:
    - The textbook's markdown files are read and split into `DocumentChunk`s.
    - For each chunk, a vector embedding is generated using an embedding model (e.g., from OpenAI).
    - The `DocumentChunk`'s text and metadata are saved to the Neon Postgres database.
    - The corresponding `VectorEmbedding` and its payload are saved to the Qdrant vector store.

2.  **Retrieval**:
    - A user's query is converted into a vector embedding.
    - This query vector is used to perform a similarity search in Qdrant.
    - Qdrant returns a list of the most similar `VectorEmbedding`s.
    - The `chunk_id`s from the results are used to look up the full `DocumentChunk` content from Postgres if needed (though the payload in Qdrant should suffice for generating the response).
    - The retrieved content is passed to a large language model along with the original query to generate a final answer.
