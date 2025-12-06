from pydantic import BaseModel, Field
from typing import List, Dict, Any
import uuid

class DocumentChunk(BaseModel):
    chunk_id: uuid.UUID = Field(default_factory=uuid.uuid4)
    document_id: str
    content: str
    metadata: Dict[str, Any]

class VectorEmbedding(BaseModel):
    vector_id: uuid.UUID
    vector: List[float]
    payload: Dict[str, Any]
