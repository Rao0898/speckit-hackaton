@app.post("/rag-answer")
def rag_answer(query: str):
    # First, search for relevant documents using the /search endpoint logic
    search_results = search_documents(query) # Reuse the search logic

    if not search_results:
        return {"answer": "I could not find relevant information in the textbook.", "source_chunks": []}

    # Extract content from the search results to use as context
    context = "\n\n".join([chunk.content for chunk in search_results])

    # Use OpenAI to generate an answer based on the context
    response = client.chat.completions.create(
        model="gpt-4o", # or gpt-3.5-turbo
        messages=[
            {"role": "system", "content": "You are a helpful assistant that answers questions based on the provided context from a technical textbook. If you cannot find the answer in the context, state that you don't know."},
            {"role": "user", "content": f"Context: {context}\n\nQuestion: {query}"}
        ]
    )

    answer = response.choices[0].message.content
    return {"answer": answer, "source_chunks": search_results}

# Ensure to add this at the beginning of the file if not already present
from typing import List
