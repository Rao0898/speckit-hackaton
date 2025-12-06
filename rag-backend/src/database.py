import os
from qdrant_client import QdrantClient
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

db_service = DatabaseService()
