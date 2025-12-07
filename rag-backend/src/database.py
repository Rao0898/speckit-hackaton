import os
from qdrant_client import QdrantClient
from qdrant_client.http.models import Distance, VectorParams
import psycopg2
from dotenv import load_dotenv

load_dotenv()

class DatabaseService:
    def __init__(self):
        self.qdrant_client = QdrantClient(
            url=os.getenv("QDRANT_URL"),
            api_key=os.getenv("QDRANT_API_KEY")
        )
        self.postgres_conn = psycopg2.connect(os.getenv("POSTGRES_URL"))

    def get_qdrant_client(self):
        return self.qdrant_client

    def get_postgres_cursor(self):
        return self.postgres_conn.cursor()

    def close_postgres_connection(self):
        self.postgres_conn.close()

    def create_collections_and_tables(self):
        # Create Qdrant collection
        collection_name = "textbook"
        vector_size = 768 # models/embedding-001 output dimension

        if not self.qdrant_client.collection_exists(collection_name=collection_name):
            self.qdrant_client.create_collection(
                collection_name=collection_name,
                vectors_config=VectorParams(size=vector_size, distance=Distance.COSINE)
            )
            print(f"Qdrant collection '{collection_name}' created.")
        else:
            print(f"Qdrant collection '{collection_name}' already exists.")

        # Create Postgres table
        cursor = self.postgres_conn.cursor()
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS document_chunks (
                chunk_id UUID PRIMARY KEY,
                document_id TEXT NOT NULL,
                content TEXT NOT NULL,
                metadata JSONB
            );
        """)
        self.postgres_conn.commit()
        cursor.close()
        print("Postgres table 'document_chunks' ensured to exist.")

db_service = DatabaseService()
