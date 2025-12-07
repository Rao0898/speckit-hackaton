# Research & Decisions

**Date**: 2025-12-06
**Context**: Implementation plan for the "Physical AI & Humanoid Robotics" textbook.

This document records the key research findings and architectural decisions made during the planning phase.

## 1. Docusaurus for Textbook Authoring

- **Decision**: Use Docusaurus as the static site generator for the textbook.
- **Rationale**: Docusaurus is designed for content-rich documentation sites. It provides a clean, readable format, supports MDX for interactive components, and has built-in features for versioning and search, which are ideal for a textbook. It's also a requirement of the hackathon.
- **Alternatives considered**:
  - **GitBook**: Good for documentation, but less flexible for custom components and styling compared to Docusaurus.
  - **Jupyter Book**: Excellent for code-heavy books, but the desired output is a more traditional textbook format with integrated labs, not just a series of notebooks.

## 2. RAG Chatbot Architecture

- **Decision**: Implement a RAG chatbot using a FastAPI backend, Qdrant for the vector store, and Neon Postgres for metadata.
- **Rationale**: This stack provides a robust and scalable solution for a RAG system.
  - **FastAPI**: A high-performance Python web framework, perfect for building the API layer that will serve the chatbot.
  - **Qdrant**: A specialized vector database built for speed and efficiency in similarity search, which is the core of RAG.
  - **Neon Postgres**: A serverless Postgres provider that is easy to set up and manage, suitable for storing document metadata (e.g., chapter, section) linked to the vector embeddings.
- **Alternatives considered**:
  - **All-in-one solutions (e.g., Pinecone with integrated metadata)**: While simpler, separating the vector store from the relational metadata store provides more flexibility for complex queries and future expansion.
  - **Using Postgres with pgvector**: A viable option, but a dedicated vector database like Qdrant is generally more performant for large-scale vector search.

## 3. Deployment Strategy

- **Decision**: Deploy the Docusaurus site to GitHub Pages and the FastAPI backend to a cloud service like Vercel or Railway.
- **Rationale**:
  - **GitHub Pages**: Offers free, simple, and integrated hosting for static sites directly from a GitHub repository. Docusaurus has excellent first-party support for deploying to GitHub Pages.
  - **Vercel/Railway**: These platforms offer a seamless deployment experience for modern web applications and APIs, with CI/CD integration and serverless functions that are ideal for a FastAPI backend.
- **Alternatives considered**:
  - **Self-hosting on a VPS**: More complex and requires manual setup and maintenance, which is not ideal for a hackathon timeline.
  - **AWS/GCP**: Powerful but also more complex to configure for a simple backend API.