import os
import sys
import hashlib
from pathlib import Path
from sqlalchemy.orm import Session

from ...core.database import SessionLocal, engine
from ...models import db
from ...services.content_chunk_dao import ContentChunkDAO
from ...core.qdrant import qdrant_client
from qdrant_client.models import VectorParams, Distance, PointStruct
from ...core.logger import logger
from ...core.embeddings import get_embedding

# Define collection name for Qdrant
COLLECTION_NAME = "textbook_content"

def get_text_chunks(text, max_tokens=1000):
    """Splits text into chunks of a maximum number of tokens."""
    words = text.split()
    chunks = []
    for i in range(0, len(words), max_tokens):
        chunk_words = words[i : i + max_tokens]
        chunks.append(" ".join(chunk_words))
    return chunks

def ingest_content():
    """
    Ingests content from the docs directory into the database and vector store.
    """
    logger.info("Starting content ingestion process.")
    db_session: Session = SessionLocal()
    dao = ContentChunkDAO(db_session)
    
    # Ensure Qdrant collection exists
    try:
        collection_info = qdrant_client.get_collection(collection_name=COLLECTION_NAME)
        logger.info(f"Qdrant collection '{COLLECTION_NAME}' already exists.")
        # Check if vector size is correct
        if collection_info.vectors_config.params.size != 768:
            logger.warning(f"Qdrant collection '{COLLECTION_NAME}' has incorrect vector size. Deleting and recreating.")
            qdrant_client.delete_collection(collection_name=COLLECTION_NAME)
            raise Exception("Recreating collection with correct vector size.")
    except Exception:
        qdrant_client.recreate_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=VectorParams(size=768, distance=Distance.COSINE),
        )
        logger.info(f"Created Qdrant collection '{COLLECTION_NAME}' with vector size 768.")

    # Path to the docs directory, assuming script is run from rag-backend
    docs_path = Path(__file__).parent.parent.parent.parent / "docs"
    
    new_chunks_to_embed = []

    for md_file in docs_path.glob("**/*.md"):
        logger.info(f"Processing file: {md_file.name}")
        with open(md_file, "r", encoding="utf-8") as f:
            content = f.read()
        
        chunks = get_text_chunks(content)
        
        for chunk_text in chunks:
            content_hash = hashlib.md5(chunk_text.encode("utf-8")).hexdigest()
            
            existing_chunk = dao.get_chunk_by_hash(content_hash)
            if not existing_chunk:
                new_chunk_data = {
                    "content_hash": content_hash,
                    "source_file": str(md_file.relative_to(docs_path.parent)),
                    "text": chunk_text,
                }
                new_chunks_to_embed.append(new_chunk_data)

    if new_chunks_to_embed:
        logger.info(f"Found {len(new_chunks_to_embed)} new chunks to process.")
        # Create chunks in DB
        created_db_chunks = dao.bulk_create_chunks(new_chunks_to_embed)
        
        # Generate embeddings and prepare for Qdrant
        points_to_upsert = []
        for chunk in created_db_chunks:
            embedding = get_embedding(chunk.text)
            point = PointStruct(
                id=str(chunk.id),
                vector=embedding,
                payload={
                    "source_file": chunk.source_file,
                    "text": chunk.text
                }
            )
            points_to_upsert.append(point)

        # Upsert to Qdrant
        if points_to_upsert:
            qdrant_client.upsert(
                collection_name=COLLECTION_NAME,
                points=points_to_upsert,
                wait=True,
            )
            logger.info(f"Upserted {len(points_to_upsert)} new vectors to Qdrant.")
    else:
        logger.info("No new content to ingest.")

    db_session.close()
    logger.info("Content ingestion process finished.")

if __name__ == "__main__":
    # Create tables if they don't exist
    db.Base.metadata.create_all(bind=engine)
    ingest_content()
