import google.generativeai as genai

from ..core.qdrant import qdrant_client
from ..models.query import QueryResponse, Citation, QuerySelectionRequest
from ..core.logger import logger
from ..core.embeddings import get_embedding

# Initialize Gemini client for chat completions
model = genai.GenerativeModel('gemini-pro')
COLLECTION_NAME = "textbook_content"

class RAGService:
    def query_general(self, query: str) -> QueryResponse:
        """
        Handles a general query by retrieving context and generating an answer.
        """
        logger.info(f"Received general query: {query}")
        query_embedding = get_embedding(query)

        # Search for relevant context in Qdrant
        search_results = qdrant_client.search(
            collection_name=COLLECTION_NAME,
            query_vector=query_embedding,
            limit=3,  # Return top 3 most relevant chunks
        )

        context = ""
        citations = []
        for result in search_results:
            context += result.payload["text"] + "\n\n"
            citations.append(
                Citation(
                    source_file=result.payload["source_file"],
                    text=result.payload["text"],
                )
            )
        logger.info(f"Retrieved {len(citations)} citations for general query.")
        # Construct the prompt
        prompt = f"""
        You are a helpful assistant for the 'Physical AI & Humanoid Robotics' textbook.
        Answer the following question based on the provided context.
        Your answer must be grounded in the context. Do not use outside knowledge.

        Context:
        ---
        {context}
        ---

        Question: {query}
        """

        # Generate the answer using Gemini
        response = model.generate_content(prompt)
        answer = response.text
        logger.info("Generated answer for general query.")
        return QueryResponse(answer=answer, citations=citations)

    def query_selection(self, request: QuerySelectionRequest) -> QueryResponse:
        """
        Handles a query based on a specific text selection.
        """
        logger.info(f"Received selection query: {request.query} with selection length {len(request.selection)}")
        context = request.selection

        # Construct the prompt
        prompt = f"""
        You are a helpful assistant. Answer the question strictly based on the provided text.
        Do not use any outside knowledge. If the answer is not in the text, say so.

        Text:
        ---
        {context}
        ---

        Question: {request.query}
        """

        # Generate the answer using Gemini
        response = model.generate_content(prompt)
        answer = response.text
        logger.info("Generated answer for selection query.")
        # For selection-based queries, the citation is the selection itself
        citations = [Citation(source_file="user_selection", text=context)]

        return QueryResponse(answer=answer, citations=citations)
