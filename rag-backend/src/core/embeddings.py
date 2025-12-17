import os
import google.generativeai as genai
from ..core.config import settings

genai.configure(api_key=settings.GEMINI_API_KEY)

def get_embedding(text: str) -> list[float]:
    """
    Generates an embedding for the given text using the Gemini API.

    Args:
        text: The text to embed.

    Returns:
        A list of floats representing the embedding.
    """
    return genai.embed_content(
        model="models/embedding-001",
        content=text,
        task_type="retrieval_document"
    )["embedding"]
