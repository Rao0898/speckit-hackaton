from pydantic import BaseModel
from typing import Optional

class TranslationRequest(BaseModel):
    text: str
    target_language: str = "ur" # Default to Urdu

class TranslationResponse(BaseModel):
    translated_text: str
