import os
import markdown
from bs4 import BeautifulSoup
from typing import List
from .database import db_service
from .models import DocumentChunk
from .gemini_config import gemini_model # Import the initialized GeminiModel
import json

def get_embedding(text):
    return gemini_model.embed_content(text=text)

def chunk_text(text, max_chars=1000):
    """
    A simple text chunking implementation based on character count.
    This can be improved with more sophisticated sentence-aware chunking.
    """
    chunks = []
    current_chunk = ""
    words = text.split()
    for word in words:
        if len(current_chunk) + len(word) + 1 <= max_chars:
            chunks.append(current_chunk.strip())
            current_chunk = word + " "
        else:
            current_chunk += (word + " ")
    if current_chunk:
        chunks.append(current_chunk.strip())
    return chunks

def ingest_markdown_files(directory):
    qdrant = db_service.get_qdrant_client()
    cursor = db_service.get_postgres_cursor()

    for root, _, files in os.walk(directory):
        for file in files:
            if file.endswith(".md") or file.endswith(".mdx"):
                filepath = os.path.join(root, file)
                with open(filepath, 'r', encoding='utf-8') as f:
                    md_content = f.read()
                
                # A simple way to extract text from markdown
                html = markdown.markdown(md_content)
                soup = BeautifulSoup(html, 'html.parser')
                text_content = soup.get_text()

                chunks = chunk_text(text_content)

                for i, chunk_text in enumerate(chunks):
                    # Create document chunk
                    doc_chunk = DocumentChunk(
                        document_id=filepath,
                        content=chunk_text,
                        metadata={"chapter": "...", "section": "..."} # Placeholder metadata
                    )

                    # Generate embedding
                    embedding = get_embedding(doc_chunk.content)

                    # Save to Postgres
                    cursor.execute(
                        "INSERT INTO document_chunks (chunk_id, document_id, content, metadata) VALUES (%s, %s, %s, %s)",
                        (str(doc_chunk.chunk_id), doc_chunk.document_id, doc_chunk.content, json.dumps(doc_chunk.metadata))
                    )

                    # Save to Qdrant
                    qdrant.upsert(
                        collection_name="textbook",
                        points=[{
                            "id": str(doc_chunk.chunk_id),
                            "vector": embedding,
                            "payload": doc_chunk.metadata
                        }]
                    )
    
    db_service.postgres_conn.commit()
    cursor.close()
    print("Ingestion complete.")

def search_documents(query: str, top_k: int = 5) -> List[DocumentChunk]:
    query_embedding = gemini_model.embed_content(text=query, task_type="RETRIEVAL_QUERY")
    qdrant = db_service.get_qdrant_client()

    search_result = qdrant.search(
        collection_name="textbook",
        query_vector=query_embedding,
        limit=top_k
    )

    chunk_ids = [hit.id for hit in search_result]
    if not chunk_ids:
        return []

    # Fetch full DocumentChunk data from Postgres
    cursor = db_service.get_postgres_cursor()
    placeholders = ','.join(['%s'] * len(chunk_ids))
    cursor.execute(
        f"SELECT chunk_id, document_id, content, metadata FROM document_chunks WHERE chunk_id IN ({placeholders})",
        tuple(str(cid) for cid in chunk_ids)
    )
    db_chunks_data = cursor.fetchall()
    cursor.close()

    retrieved_chunks = []
    for row in db_chunks_data:
        chunk_id, document_id, content, metadata = row
        retrieved_chunks.append(DocumentChunk(
            chunk_id=chunk_id,
            document_id=document_id,
            content=content,
            metadata=json.loads(metadata) # Assuming metadata is stored as JSON string
        ))
    
    return retrieved_chunks


if __name__ == "__main__":
    # This is an example of how to run the ingestion
    # You would need to create the 'textbook' collection in Qdrant first
    # and a 'document_chunks' table in Postgres.
    # ingest_markdown_files("../../my-book/docs") 
    print("This script is intended to be imported, not run directly yet.")
