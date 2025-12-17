import os
from functools import lru_cache
from pydantic_settings import BaseSettings
from dotenv import load_dotenv

load_dotenv()

class Settings(BaseSettings):
    # OpenAI
    OPENAI_API_KEY: str = os.getenv("OPENAI_API_KEY")

    # Qdrant
    QDRANT_URL: str = os.getenv("QDRANT_URL")
    QDRANT_API_KEY: str = os.getenv("QDRANT_API_KEY")

    # Neon (Postgres)
    DATABASE_URL: str = os.getenv("DATABASE_URL")

    class Config:
        case_sensitive = True


@lru_cache()
def get_settings() -> Settings:
    return Settings()

settings = get_settings()
