# Data Model: RAG Chatbot Backend

**Date**: 2025-12-15
**Feature**: [RAG Chatbot Backend](../spec.md)

This document outlines the data models for the entities required by the RAG chatbot backend. The design is based on the [feature specification](../spec.md).

## Core Entities

### `ContentChunk`

This is the primary entity for the RAG system. It represents a piece of content from the textbook that has been processed and is ready for embedding and retrieval.

**Persistence**: Metadata for `ContentChunk` will be stored in the Neon Serverless Postgres database. The vector embedding will be stored in Qdrant.

**Fields**:

| Field Name      | Data Type       | Description                                                                                             | Nullable |
|-----------------|-----------------|---------------------------------------------------------------------------------------------------------|----------|
| `id`            | UUID            | Unique identifier for the content chunk. Will be used as the primary key.                               | No       |
| `content_hash`  | STRING          | An MD5 hash of the `text` field to detect content changes and avoid re-processing identical chunks.      | No       |
| `source_file`   | STRING          | The path to the source Markdown file from which the chunk was extracted (e.g., `docs/01-intro.md`).      | No       |
| `text`          | TEXT            | The actual text content of the chunk.                                                                   | No       |
| `metadata`      | JSONB           | A flexible field to store any additional metadata, such as section headers or other structural info.    | Yes      |
| `created_at`    | TIMESTAMP       | The timestamp when the chunk was first created.                                                         | No       |
| `updated_at`    | TIMESTAMP       | The timestamp when the chunk was last updated.                                                          | No       |

**Relationships**:

-   A `ContentChunk` has a one-to-one relationship with a vector embedding stored in Qdrant, linked by the `id`.

## State Transitions

-   A `ContentChunk` is created during the ingestion process.
-   It is updated if the source Markdown file changes and the `content_hash` for that chunk is different.
-   It can be deleted if the source file is removed.
