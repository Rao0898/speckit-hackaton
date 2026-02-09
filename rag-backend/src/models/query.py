from pydantic import BaseModel
from typing import List, Optional

class QueryRequest(BaseModel):
    query: str
    language: Optional[str] = "en"

class Citation(BaseModel):
    source_file: str
    text: str

class QueryResponse(BaseModel):
    answer: str
    citations: List[Citation]

class QuerySelectionRequest(BaseModel):
    query: str
    selection: str
