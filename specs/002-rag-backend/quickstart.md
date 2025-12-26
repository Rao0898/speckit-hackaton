# Quickstart: RAG Chatbot Backend

This guide provides step-by-step instructions on how to set up and run the RAG (Retrieval-Augmented Generation) Chatbot Backend. This backend uses Voyage AI for embeddings, Qdrant Cloud for vector storage, and a Gemini-powered agent for conversational reasoning.

## Prerequisites

Before you begin, ensure you have the following:

-   Python 3.8+ installed.
-   `pip` (Python package installer)
-   Access to API keys for:
    -   **Voyage AI**: For generating text embeddings.
    -   **Qdrant Cloud**: A vector database for storing and retrieving embeddings. You'll need a cluster URL and an API key.
    -   **Gemini API**: For the conversational agent (model: `gemini-1.5-flash`).
-   Familiarity with command-line interface (CLI).

## Step 1: Clone the Repository (if not already done)

If you haven't already, clone the project repository:

```bash
git clone <your_repository_url>
cd humanoid-book # Or your project's root directory
```

## Step 2: Set Up Your Environment

1.  **Navigate to the project root**:
    ```bash
    cd D:\hackaton-specify\humanoid-book # Replace with your actual project root
    ```

2.  **Create and configure the `.env` file**:
    The project uses a `.env` file to manage sensitive API keys and configurations.
    Create a file named `.env` in the root directory of your project if it doesn't already exist.

    Open the `.env` file and add your API keys and Qdrant URL as follows. Replace the placeholder values with your actual credentials:

    ```env
    # .env file for rag_chatbot_agent.py

    # Voyage AI API Key
    VOYAGE_API_KEY="YOUR_VOYAGE_API_KEY"

    # Qdrant Cloud API Key and URL
    QDRANT_API_KEY="YOUR_QDRANT_API_KEY"
    QDRANT_URL="YOUR_QDRANT_CLUSTER_URL"

    # Gemini API Key (from Google AI Studio or GCP)
    GEMINI_API_KEY="YOUR_GEMINI_API_KEY"
    ```

    **Important**: Do not commit your `.env` file to version control. It typically contains sensitive information. The provided `.gitignore` should already prevent this.

## Step 3: Install Dependencies

Install the required Python packages using `pip`:

```bash
pip install "openai-agents" "voyageai" "qdrant-client" "python-dotenv"
```

## Step 4: Run the RAG Chatbot Backend

1.  **Execute the Python script**:
    The main script for the RAG chatbot is `rag_chatbot_agent.py`. Run it from your project root directory:

    ```bash
    python rag_chatbot_agent.py
    ```

2.  **Initial Ingestion**:
    The first time you run the script, it will perform a simulated ingestion of textbook chapters into Qdrant. You will see output similar to:
    ```
    Starting textbook ingestion process...
    Creating collection 'textbook_collection'.
    Generating embeddings with Voyage AI...
    Upserting 3 points to Qdrant collection 'textbook_collection'...
    Ingestion complete.
    ```
    If the collection already exists and contains data, it will skip ingestion:
    ```
    Starting textbook ingestion process...
    Collection 'textbook_collection' already exists.
    Textbook content already ingested. Skipping ingestion.
    ```

3.  **Interact with the Chatbot**:
    Once the ingestion (or check) is complete, the chatbot will be ready for your queries:
    ```
    --- RAG Chatbot is Ready ---
    Ask a question about the textbook. Type 'exit' to quit.

    User Query:
    ```
    Type your question (e.g., "What is kinematics?") and press Enter.

4.  **Receive Responses**:
    The agent will process your query, retrieve relevant context, and provide an answer.
    ```
    User Query: What is kinematics?
    Agent is thinking...

    Agent Response: Kinematics is the study of motion without considering the forces that cause it.
    ```
    If the answer is not found in the selected text, it will respond with:
    ```
    Agent Response: Not in selected text
    ```

5.  **Exit the Chatbot**:
    To stop the chatbot, type `exit` at the `User Query:` prompt and press Enter.

You have successfully set up and interacted with the RAG Chatbot Backend!