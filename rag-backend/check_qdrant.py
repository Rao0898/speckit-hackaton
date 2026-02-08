import os
import socket
import traceback
from urllib.parse import urlparse
from dotenv import dotenv_values, find_dotenv
from qdrant_client import QdrantClient

# --- 1. Load environment variables directly ---
dotenv_path = find_dotenv(usecwd=True) # Search from current working directory
if dotenv_path:
    print(f"Loading .env values from: {dotenv_path}")
    config = dotenv_values(dotenv_path)
else:
    print("No .env file found by find_dotenv(). Exiting.")
    exit()

qdrant_url = config.get("QDRANT_URL")
qdrant_api_key = config.get("QDRANT_API_KEY")
collection_name = "Voyage-embediing-collection" # Correct collection name

print(f"QDRANT_URL (from config): {qdrant_url}")
print("-" * 20)

if not qdrant_url or "YOUR_QDRANT_CLUSTER_URL" in qdrant_url:
    print("QDRANT_URL is not set or contains placeholder value. Exiting.")
    exit()

# --- 2. Test DNS Resolution ---
try:
    hostname = urlparse(qdrant_url).hostname
    print(f"Attempting to resolve hostname: {hostname}")
    ip_address = socket.gethostbyname(hostname)
    print(f"Successfully resolved {hostname} to {ip_address}")
except Exception as e:
    print(f"Error resolving hostname: {e}")
    print("DNS resolution failed from within Python. This is likely the root cause.")
    exit()

print("-" * 20)

# --- 3. Test Qdrant Connection ---
print("Attempting to connect to Qdrant...")
try:
    qdrant = QdrantClient(
        url=qdrant_url, 
        api_key=qdrant_api_key,
        timeout=10  # Add a timeout
    )
    
    collection_info = qdrant.get_collection(collection_name=collection_name)
    print("Successfully connected to Qdrant!")
    print(f"\nCollection '{collection_name}' info:")
    print(collection_info)

except Exception as e:
    print(f"An error occurred while connecting to Qdrant or getting collection info.")
    print("\n--- Full Traceback ---")
    traceback.print_exc()
    print("----------------------")