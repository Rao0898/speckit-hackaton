from sentence_transformers import SentenceTransformer

model = SentenceTransformer("all-MiniLM-L6-v2")

def get_embedding(text: str) -> list[float]:
    """
    Generates an embedding for the given text using a local SentenceTransformer model.

    Args:
        text: The text to embed.

    Returns:
        A list of floats representing the embedding.
    """
    return model.encode(text).tolist()
