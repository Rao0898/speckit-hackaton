import os
import markdown
from bs4 import BeautifulSoup
import tiktoken
from .database import db_service
from .models import DocumentChunk
from openai import OpenAI

client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

def get_embedding(text, model="text-embedding-3-small"):
   text = text.replace("\n", " ")
   return client.embeddings.create(input = [text], model=model).data[0].embedding

def chunk_text(text, max_tokens=512):
    """
    A simple text chunking implementation.
    This can be improved with more sophisticated sentence-aware chunking.
    """
    tokenizer = tiktoken.get_encoding("cl100k_base")
    tokens = tokenizer.encode(text)
    chunks = []
    for i in range(0, len(tokens), max_tokens):
        chunk_tokens = tokens[i:i + max_tokens]
        chunks.append(tokenizer.decode(chunk_tokens))
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
                        (str(doc_chunk.chunk_id), doc_chunk.document_id, doc_chunk.content, str(doc_chunk.metadata))
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

if __name__ == "__main__":
    # This is an example of how to run the ingestion
    # You would need to create the 'textbook' collection in Qdrant first
    # and a 'document_chunks' table in Postgres.
    # ingest_markdown_files("../../my-book/docs") 
    print("This script is intended to be imported, not run directly yet.")
