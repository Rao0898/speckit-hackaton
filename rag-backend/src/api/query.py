from fastapi import APIRouter
from ..models.query import QueryRequest, QueryResponse, QuerySelectionRequest
from ..services.rag_service import RAGService

router = APIRouter()
rag_service = RAGService()

@router.post("/query", response_model=QueryResponse)
def query_general(request: QueryRequest):
    """
    Handles a general question about the textbook.
    """
    return rag_service.query_general(request.query)

@router.post("/query-selection", response_model=QueryResponse)
def query_selection(request: QuerySelectionRequest):
    """
    Handles a question about a specific text selection.
    """
    return rag_service.query_selection(request)
