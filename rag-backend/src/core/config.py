from functools import lru_cache
from pydantic_settings import BaseSettings

class Settings(BaseSettings):
    # Gemini
    GEMINI_API_KEY: str
    VOYAGE_API_KEY: str

    # Qdrant
    QDRANT_URL: str | None = None
    QDRANT_API_KEY: str | None = None
    VOYAGE_API_KEY: str 

    # Neon (Postgres)
    DATABASE_URL: str = "sqlite:///./rag.db"

    class Config:
        case_sensitive = True
        env_file = "D:/hackaton-specify/humanoid-book/rag-backend/.env"
        extra = 'ignore'



@lru_cache()
def get_settings() -> Settings:
    return Settings()

settings = get_settings()
