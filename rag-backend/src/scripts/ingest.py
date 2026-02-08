import os
import sys
import time  # <--- Added this
from pathlib import Path
from sqlalchemy.orm import Session
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance
from src.core.config import settings # Keep settings for other values
from dotenv import dotenv_values, find_dotenv

# Directly load VOYAGE_API_KEY for placeholder check
dotenv_path = find_dotenv(usecwd=True)
if dotenv_path:
    config_from_dotenv = dotenv_values(dotenv_path)
    VOYAGE_API_KEY_DIRECT = config_from_dotenv.get("VOYAGE_API_KEY")
else:
    VOYAGE_API_KEY_DIRECT = None
from src.core.embeddings import get_embedding
from src.core.database import SessionLocal
from src.services.content_chunk_dao import ContentChunkDAO

# Constants
VECTOR_SIZE = 1024
COLLECTION_NAME = "Voyage-embediing-collection"
CONTENT_DIR = Path(__file__).resolve().parent.parent.parent.parent / "my-book" / "docs"

# Initialize Qdrant client
qdrant_url = settings.QDRANT_URL
qdrant_api_key = settings.QDRANT_API_KEY

if not qdrant_url:
    print("Error: QDRANT_URL is not set.")
    sys.exit(1)

if "YOUR_VOYAGE_API_KEY" in settings.VOYAGE_API_KEY:
    print("Error: Placeholder API keys found in .env file.")
    sys.exit(1)

qdrant = QdrantClient(
    url=qdrant_url,
    api_key=qdrant_api_key,
    timeout=60,
    verify=False if "localhost" in qdrant_url or "127.0.0.1" in qdrant_url else True
)

# Ensure collection exists
try:
    if not qdrant.collection_exists(collection_name=COLLECTION_NAME):
        qdrant.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=VectorParams(size=VECTOR_SIZE, distance=Distance.COSINE)
        )
except Exception as e:
    print(f"Error checking or creating collection: {e}")
    sys.exit(1)

# DB session
db: Session = SessionLocal()
dao = ContentChunkDAO(db)

# Ingest content
print(f"Checking for markdown files in: {CONTENT_DIR}")
markdown_files = sorted(CONTENT_DIR.glob("*.md"))

if not markdown_files:
    print(f"No markdown files found in {CONTENT_DIR}.")
    sys.exit(0)

print(f"Found {len(markdown_files)} markdown files to process.")

for file_path in markdown_files:
    print(f"Processing: {file_path.name}")
    try:
        with open(file_path, "r", encoding="utf-8") as f:
            text = f.read()

        if not text.strip():
            continue

        import hashlib
        content_hash = hashlib.md5(text.encode()).hexdigest()

        if dao.get_chunk_by_hash(content_hash):
            print(f"Skipping {file_path.name}, already ingested.")
            continue

        # Get embedding with OpenAI
        embedding = get_embedding(text)

        if embedding is None:
            print(f"Could not generate embedding for {file_path.name}. Skipping.")
            continue

        # Store in DB
        dao.create_chunk({
            "content_hash": content_hash,
            "source_file": file_path.name,
            "text": text,
        })

        # Upload to Qdrant
        from qdrant_client.models import PointStruct
        qdrant.upsert(
            collection_name=COLLECTION_NAME,
            points=[
                PointStruct(
                    id=content_hash,
                    vector=embedding,
                    payload={"source_file": file_path.name, "text": text}
                )
            ],
            wait=True
        )
        print(f"Successfully ingested and indexed {file_path.name}")
        
        # --- WAIT TO AVOID RATE LIMITS ---
        time.sleep(20) # 2 seconds ka delay har file ke baad

    except Exception as e:
        print(f"Error processing {file_path.name}: {e}")

print("Content ingestion finished!")