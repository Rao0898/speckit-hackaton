import voyageai
from src.core.config import settings
import time

# Voyage Client initialize karein
client = voyageai.Client(api_key=settings.VOYAGE_API_KEY)

def get_embedding(text: str, model: str = "voyage-3") -> list[float]:
    """
    Voyage AI use karke embedding generate karen.
    """
    retries = 3
    for i in range(retries):
        try:
            # Voyage ka sahi tareeqa: client.embed()
            # OpenAI ki tarah .embeddings.create nahi hota yahan
            response = client.embed(
                [text], 
                model=model, 
                input_type="document"
            )
            
            # Voyage mein response.embeddings[0] se data milta hai
            return response.embeddings[0]
            
        except Exception as e:
            # Yahan se 'openai' ka zikr bilkul khatam kar diya hai
            print(f"Error occurred with Voyage: {e}")
            
            if i < retries - 1:
                wait_time = 2 ** i
                print(f"Retrying in {wait_time}s...")
                time.sleep(wait_time)
            else:
                # Agar saare retries khatam ho jayein
                return None
                
    return None