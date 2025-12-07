import os
import google.generativeai as genai
from dotenv import load_dotenv

load_dotenv() # Load environment variables from .env file

class GeminiModel:
    def __init__(self, model_name="gemini-pro"):
        self.model_name = model_name
        self._configure_gemini()
        self.model = genai.GenerativeModel(self.model_name)

    def _configure_gemini(self):
        GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
        if not GEMINI_API_KEY:
            raise ValueError("GEMINI_API_KEY environment variable not set.")
        genai.configure(api_key=GEMINI_API_KEY)

    def generate_content(self, prompt: str, **kwargs):
        return self.model.generate_content(prompt, **kwargs)

    def embed_content(self, text: str, task_type="RETRIEVAL_DOCUMENT", title=None):
        if not text:
            return []
        
        # Ensure the text is not an empty string, as it causes an error with the API
        # The API expects a list of strings for batch embedding, but we're doing one at a time here.
        # It also explicitly says 'content' should not be empty.
        
        embedding_config = {
            "model": "models/embedding-001",
            "content": [text], # Embeddings API expects a list of strings
            "task_type": task_type,
        }
        if title:
            embedding_config["title"] = title

        response = genai.embed_content(**embedding_config)
        return response['embedding']

# Initialize Gemini Model once for reuse
gemini_model = GeminiModel()

def get_gemini_api_key():
    GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
    if not GEMINI_API_KEY:
        raise ValueError("GEMINI_API_KEY environment variable not set.")
    return GEMINI_API_KEY

