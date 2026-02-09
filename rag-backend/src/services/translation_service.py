import os
from google import genai
from ..core.config import settings
from ..core.logger import logger

# FORCE: Use the key strictly from our settings (.env)
client = genai.Client(api_key=settings.GEMINI_API_KEY)

class TranslationService:
    async def translate_text(self, text: str, target_language: str) -> str:
        if target_language.lower() == "ur":
            prompt = f"""
            Translate the following English text to academic Urdu.
            Maintain the original formatting (e.g., Markdown, HTML tags, line breaks).
            Keep all technical terms (e.g., "ROS 2", "Nodes", "Docker", "Python", "JavaScript", "FastAPI", "API", "CPU", "GPU", "AI", "ML", "RL", "NLP", "Computer Vision", "Robotics", "Kinematics", "Dynamics", "Locomotion", "Manipulation", "Simulation", "Gazebo", "Isaac Sim", "URDF", "SDF", "Robot Operating System", "Neural Networks", "Deep Learning", "Reinforcement Learning") in English script.
            Explain the context of these technical terms briefly in Urdu if necessary for clarity.
            Ensure the translation is contextual and easy to understand for an academic audience.

            English Text:
            ---
            {text}
            ---
            """
        else:
            # For other languages, or if no translation is needed, return original text (or implement other translation logic)
            return text
        
        try:
            response = await client.models.generate_content_async(
                model="gemini-1.5-flash", # Using async version
                contents=prompt
            )
            return response.text
        except Exception as e:
            logger.error(f"Gemini Translation Error: {e}")
            return f"Translation failed: {e}"
