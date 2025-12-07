from fastapi import FastAPI, HTTPException, status, Body  
from typing import List, Optional
from pydantic import BaseModel
import uvicorn
import os
from dotenv import load_dotenv

# Import necessary components from our backend
from src.ingest import search_documents, ingest_markdown_files
from src.database import db_service
from src.gemini_config import gemini_model
from src.models import DocumentChunk  # Assuming DocumentChunk is defined here

load_dotenv()  # Load environment variables

app = FastAPI()

# -----------------------------
# Add CORS middleware so frontend fetch works
from fastapi.middleware.cors import CORSMiddleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # or ["http://localhost:3000"] for frontend only
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
# -----------------------------

# Create Qdrant collection and Postgres table on startup
db_service.create_collections_and_tables()

SECRET_KEY = os.getenv("SECRET_KEY")

class SearchResult(BaseModel):
    document_id: str
    content: str
    metadata: dict
    score: float = None

# ✅ ROOT ROUTE
@app.get("/")
def root():
    return {"message": "Server is running"}

# ✅ Updated /rag-answer route with Body(...) to fix 422 error
@app.post("/rag-answer")
def rag_answer(
    query: str = Body(..., embed=True), 
    context: Optional[str] = Body(None, embed=True)
):
    try:
        # 1️⃣ Prepare search results
        search_results = []
        if context:
            full_query = f"Based on the following text: '{context}', {query}"
        else:
            full_query = query
            search_results = search_documents(full_query) or []

        # 2️⃣ If nothing found, fallback generic context
        retrieved_content = "\n\n".join([chunk.content for chunk in search_results])
        final_context = ""
        if retrieved_content:
            final_context += f"Retrieved information: {retrieved_content}\n\n"
        if context:
            final_context += f"User-selected context: {context}\n\n"
        if not final_context:
            final_context = "No context available, please answer based on general knowledge.\n\n"

        # 3️⃣ Prepare prompt for Gemini model
        messages = [
            {"role": "user", "parts": [{"text": "You are a helpful assistant that answers questions based on the provided context from a technical textbook. If you cannot find the answer in the context, state that you don't know."}]},
            {"role": "model", "parts": [{"text": "Okay, I will do my best to answer based on the context."}]},
            {"role": "user", "parts": [{"text": f"Context: {final_context}\nQuestion: {query}"}]}
        ]

        # 4️⃣ Generate content from Gemini model
        response = gemini_model.generate_content(prompt=messages)
        answer = getattr(response, "text", None)

        # 5️⃣ Ensure answer is never None
        if not answer:
            answer = "I could not generate an answer."

        return {"answer": answer, "source_chunks": search_results or []}

    except Exception as e:
        print("Error in /rag-answer:", e)
        return {"answer": "Sorry, an error occurred on the server.", "source_chunks": []}

@app.post("/ingest")
def ingest_data(docs_path: str = "../../my-book/docs"):
    try:
        ingest_markdown_files(docs_path)
        return {"message": f"Successfully ingested markdown files from {docs_path}"}
    except Exception as e:
        raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail=str(e))

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
