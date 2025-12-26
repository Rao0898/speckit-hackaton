# rag_chatbot_agent.py

import os
import uuid
import asyncio
from dotenv import load_dotenv
from typing import List, Dict, Any

# Qdrant client for vector database
import qdrant_client
from qdrant_client.http import models

# Voyage AI for embeddings
import voyageai

# OpenAI Agents for the agent framework
from openai_agents import Tool, Assistant
from openai_agents.client import Gemini

# --- Environment and API Key Setup ---
# Load environment variables from .env file for API keys
load_dotenv()

VOYAGE_API_KEY = os.getenv("VOYAGE_API_KEY")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")

# --- Qdrant and Voyage AI Configuration ---
# Initialize clients
if not VOYAGE_API_KEY:
    raise ValueError("VOYAGE_API_KEY not found in .env file")
vo = voyageai.Client(api_key=VOYAGE_API_KEY)

if not QDRANT_API_KEY or not QDRANT_URL:
    raise ValueError("QDRANT_API_KEY or QDRANT_URL not found in .env file")
qdrant_client = qdrant_client.QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

# Define collection name for Qdrant
COLLECTION_NAME = "textbook_collection"
# Voyage AI's embedding model and its dimension
VOYAGE_EMBEDDING_MODEL = "voyage-2"
VOYAGE_EMBEDDING_DIMENSION = 1024

# --- Text Ingestion (Placeholder) ---
def get_or_create_collection(client: qdrant_client.QdrantClient, collection_name: str):
    """Creates a Qdrant collection if it doesn't exist."""
    try:
        client.get_collection(collection_name=collection_name)
        print(f"Collection '{collection_name}' already exists.")
    except Exception:
        print(f"Creating collection '{collection_name}'.")
        client.create_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(
                size=VOYAGE_EMBEDDING_DIMENSION,
                distance=models.Distance.COSINE
            ),
        )

async def ingest_textbook_chapters():
    """
    Placeholder function to simulate fetching, chunking, embedding,
    and storing textbook content.
    """
    print("Starting textbook ingestion process...")
    get_or_create_collection(qdrant_client, COLLECTION_NAME)

    # Dummy textbook content
    chapters = {
        "Chapter 1: Introduction to Robotics": "Robotics is an interdisciplinary branch of engineering and science...",
        "Chapter 2: Kinematics": "Kinematics is the study of motion without considering the forces that cause it...",
        "Chapter 3: Sensors in Robotics": "Sensors are crucial for a robot to perceive its environment. Common sensors include cameras, LiDAR, and IMUs."
    }

    # Check if data is already ingested
    if qdrant_client.count(collection_name=COLLECTION_NAME).count > 0:
        print("Textbook content already ingested. Skipping ingestion.")
        return

    documents = []
    for title, text in chapters.items():
        documents.append({"title": title, "text": text})

    # Generate embeddings using Voyage AI
    print("Generating embeddings with Voyage AI...")
    embeddings = vo.embed(
        texts=[doc["text"] for doc in documents], 
        model=VOYAGE_EMBEDDING_MODEL
    ).embeddings

    # Prepare points for Qdrant
    points = [
        models.PointStruct(
            id=str(uuid.uuid4()),
            vector=embedding,
            payload={"text": doc["text"], "title": doc["title"]}
        )
        for doc, embedding in zip(documents, embeddings)
    ]

    # Upsert points to Qdrant
    print(f"Upserting {len(points)} points to Qdrant collection '{COLLECTION_NAME}'...")
    qdrant_client.upsert(
        collection_name=COLLECTION_NAME,
        points=points,
        wait=True
    )
    print("Ingestion complete.")


# --- Retrieval Function ---
def retrieve_context(query: str, top_k: int = 3) -> List[Dict[str, Any]]:
    """
    Retrieves relevant context from Qdrant for a given query.
    """
    print(f"Retrieving context for query: '{query}'")
    # Generate an embedding for the query using Voyage AI
    query_embedding = vo.embed(
        texts=[query], 
        model=VOYAGE_EMBEDDING_MODEL
    ).embeddings[0]

    # Search for similar vectors in Qdrant
    search_result = qdrant_client.search(
        collection_name=COLLECTION_NAME,
        query_vector=query_embedding,
        limit=top_k,
        with_payload=True
    )

    # Format the results
    context = [
        {"text": hit.payload["text"], "title": hit.payload["title"], "score": hit.score}
        for hit in search_result
    ]
    print(f"Retrieved {len(context)} context documents.")
    return context

# --- OpenAI-Compatible Tool Definition ---
class RetrievalTool(Tool):
    """
    A tool to retrieve relevant documents from the textbook vector store.
    """
    async def __call__(self, query: str) -> str:
        """
        Invoked by the agent to find information.

        Args:
            query: The user's question or topic to search for.

        Returns:
            A string containing the formatted search results.
        """
        retrieved_docs = retrieve_context(query)
        if not retrieved_docs:
            return "No relevant information found in the textbook."
        
        # Format the context for the agent
        formatted_context = "\n\n".join(
            [f"Source: {doc['title']}\nContent: {doc['text']}" for doc in retrieved_docs]
        )
        return formatted_context

# --- Agent Definition ---
def create_rag_agent() -> Assistant:
    """
    Creates and configures the RAG agent using the OpenAI Agents SDK.
    """
    print("Creating RAG agent...")
    if not GEMINI_API_KEY:
        raise ValueError("GEMINI_API_KEY not found in .env file")

    # System prompt to guide the agent's behavior
    system_prompt = (
        "You are a helpful assistant for a robotics textbook. "
        "Your role is to answer questions strictly based on the provided context. "
        "Use the `RetrievalTool` to find relevant information from the textbook. "
        "If the answer cannot be found in the retrieved context, you MUST respond with the exact phrase: "
        "'Not in selected text'."
    )

    # Create the agent with Gemini client and the retrieval tool
    agent = Assistant(
        client=Gemini(api_key=GEMINI_API_KEY),
        model="gemini-1.5-flash",
        system_prompt=system_prompt,
        tools=[RetrievalTool()],
    )
    print("Agent created successfully.")
    return agent

# --- Main Execution Block ---
async def main():
    """

    Main function to run the RAG chatbot CLI.
    """
    print("Initializing RAG Chatbot Agent...")
    
    # 1. Ingest textbook data into the vector store
    await ingest_textbook_chapters()
    
    # 2. Create the RAG agent
    rag_agent = create_rag_agent()

    print("\n--- RAG Chatbot is Ready ---")
    print("Ask a question about the textbook. Type 'exit' to quit.")

    # 3. CLI input loop
    while True:
        try:
            user_query = input("\nUser Query: ")
            if user_query.lower() == 'exit':
                print("Exiting chatbot.")
                break
            
            # 4. Query the agent and get the response
            print("Agent is thinking...")
            response = await rag_agent.run(user_query, stream=False)
            
            # 5. Output the answer to CLI
            print(f"\nAgent Response: {response}")

        except (KeyboardInterrupt, EOFError):
            print("\nExiting chatbot.")
            break
        except Exception as e:
            print(f"\nAn error occurred: {e}")

if __name__ == "__main__":
    asyncio.run(main())