import os
from qdrant_client import QdrantClient

# Qdrant setup
qdrant = QdrantClient(url=os.getenv("QDRANT_URL", "http://localhost:6333"))
collection_name = "text-book collection"

def check_collection():
    try:
        collection_info = qdrant.get_collection(collection_name=collection_name)
        total_vectors = collection_info['result']['vectors_count']
        print(f"Checking Qdrant collection: '{collection_name}'")
        print(f"Total vectors in collection: {total_vectors}")
        if total_vectors == 0:
            print("Collection is empty.")
        else:
            print("Collection has data!")
    except Exception as e:
        print(f"Error accessing collection: {e}")

if __name__ == "__main__":
    check_collection()
