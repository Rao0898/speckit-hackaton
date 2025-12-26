# Research: RAG Chatbot Backend

**Date**: 2025-12-15
**Feature**: [RAG Chatbot Backend](../spec.md)

## Objective

This research aims to resolve the technical choice for the reasoning framework for the RAG chatbot backend, as specified in the feature's `Assumptions` and the project `constitution.md`. The choice is between "OpenAI Agents" and "ChatKit SDK".

## Investigation

A web search for "ChatKit SDK" for chatbot development revealed two primary candidates:

1.  **OpenAI ChatKit**: A toolkit from OpenAI for building "agentic chat experiences". It provides a Python SDK, which aligns with the project's FastAPI backend. It is designed for agentic workflows, which fits the "Chatbot Reasoning Agent" role mentioned in the constitution.
2.  **ChatBotKit SDK**: A platform that is primarily JavaScript-based (Node.js, React). This does not align with the project's Python-based backend.

## Decision

We will use the **OpenAI Python SDK** and its agent-related functionalities, which seems to be what is meant by "OpenAI ChatKit" in this context.

### Rationale

-   **Technology Alignment**: The OpenAI SDK has robust Python support, which integrates seamlessly with the planned FastAPI backend.
-   **Agentic Framework**: The goal is to build a "Reasoning Agent". OpenAI's tools are explicitly designed for creating agents that can use tools, which is exactly what a RAG pipeline does (the retrieval is a tool).
-   **Ecosystem**: Using OpenAI's framework keeps the core logic within a single, well-supported ecosystem, which is beneficial for a hackathon project aiming for rapid development.
-   **Constitution Compliance**: This choice directly satisfies the "OpenAI Agents or ChatKit SDK" requirement from the constitution.

### Alternatives Considered

-   **ChatBotKit**: Rejected due to its primary focus on JavaScript, which does not fit our Python backend.
-   **LangChain/LlamaIndex**: While these are powerful and popular frameworks for RAG, the project constitution specifically mentions "OpenAI Agents or ChatKit SDK". To adhere to the constraints, we will build directly on top of OpenAI's libraries. This also reduces the number of third-party dependencies.
