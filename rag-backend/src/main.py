from fastapi import FastAPI, HTTPException, Request, status
from fastapi.responses import JSONResponse
from fastapi.exceptions import RequestValidationError
from .api import query
from .core.logger import logger
from .core.exceptions import CustomException, http_exception_handler, custom_exception_handler, validation_exception_handler

app = FastAPI(
    title="RAG Chatbot Backend",
    description="API for the Physical AI & Humanoid Robotics textbook RAG chatbot.",
    version="1.0.0",
)

app.include_router(query.router, prefix="/api/v1")

# Register exception handlers
app.add_exception_handler(HTTPException, http_exception_handler)
app.add_exception_handler(CustomException, custom_exception_handler)
app.add_exception_handler(RequestValidationError, validation_exception_handler) # Handles Pydantic validation errors
app.add_exception_handler(Exception, validation_exception_handler) # Catch-all for other exceptions


@app.get("/health", summary="Health Check")
def health_check():
    """
    Simple health check endpoint.
    """
    logger.info("Health check endpoint called.")
    return {"status": "ok"}
