from qdrant_client import QdrantClient
from .config import settings

def get_qdrant_client():
    """
    Returns a Qdrant client instance configured to connect to Qdrant Cloud.
    """
    client = QdrantClient(
        url=settings.QDRANT_URL, 
        api_key=settings.QDRANT_API_KEY
    )
    return client

qdrant_client = get_qdrant_client()
