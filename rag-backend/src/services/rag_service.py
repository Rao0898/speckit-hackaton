import os
from google import genai
from ..core.config import settings
from ..core.qdrant import qdrant_client
from ..models.query import QueryResponse, Citation, QuerySelectionRequest
from ..core.logger import logger
from ..core.embeddings import get_embedding

# FORCE: Delete any existing global GOOGLE_API_KEY from environment
if "GOOGLE_API_KEY" in os.environ:
    del os.environ["GOOGLE_API_KEY"]

# FORCE: Use the key strictly from our settings (.env)
client = genai.Client(api_key=settings.GEMINI_API_KEY)

COLLECTION_NAME = "Voyage-embediing-collection"

class RAGService:
    def query_general(self, query: str, language: str = "en") -> QueryResponse:
        logger.info(f"Received general query: {query}, language: {language}")

        # Check for greeting messages
        greetings = ["hi", "hello", "assalam-o-alaikum", "salam"]
        if query.lower() in greetings:
            if language == "ur":
                return QueryResponse(answer="وعلیکم السلام! میں آپ کی نصابی کتاب سمجھنے میں کیسے مدد کر سکتا ہوں؟", citations=[])
            else:
                return QueryResponse(answer="Hello there! How can I assist you today?", citations=[])

        query_embedding = get_embedding(query)

        search_results = qdrant_client.query_points(
            collection_name=COLLECTION_NAME,
            query=query_embedding,
            limit=5,
            with_payload=True,
        )

        context = ""
        citations = []
        for result in search_results.points:
            if result.payload and "text" in result.payload:
                context += result.payload["text"] + "\n\n"
                citations.append(
                    Citation(
                        source_file=result.payload.get("source_file", "unknown"),
                        text=result.payload["text"],
                    )
                )
        
        if not citations:
            return QueryResponse(answer="Context not found.", citations=[])

        prompt = f"Answer strictly using this context:\n{context}\n\nQuestion: {query}"
        
        try:
            # TRYING THE MOST COMPATIBLE VERSION
            response = client.models.generate_content(
                model="gemini-2.5-flash", 
                contents=prompt
            )
            answer = response.text
            logger.info("Success! Gemini responded.")
        except Exception as e:
            logger.error(f"Gemini Error: {e}")
            answer = f"Gemini is still blocked. Error: {e}"

        return QueryResponse(answer=answer, citations=citations)

    def query_selection(self, request: QuerySelectionRequest) -> QueryResponse:
        response = client.models.generate_content(
            model="gemini-2.5-flash",
            contents=f"Text: {request.selection}\nQuestion: {request.query}"
        )
        return QueryResponse(answer=response.text, citations=[Citation(source_file="selection", text=request.selection)])