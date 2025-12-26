import os
from dotenv import load_dotenv
from .core.logger import logger

# Load environment variables
load_dotenv()


from fastapi import FastAPI, HTTPException, Request, status
from fastapi.responses import JSONResponse
from fastapi.exceptions import RequestValidationError
from fastapi.middleware.cors import CORSMiddleware # Import CORSMiddleware
from .api import query
from .core.exceptions import CustomException, http_exception_handler, custom_exception_handler, validation_exception_handler

app = FastAPI(
    title="RAG Chatbot Backend",
    description="API for the Physical AI & Humanoid Robotics textbook RAG chatbot.",
    version="1.0.0",
)

# Add CORS middleware
# Best practice: use environment variables for allowed origins
origins_str = os.getenv("ALLOWED_FRONTEND_ORIGINS", "http://localhost:3000,http://127.0.0.1:3000")
origins = [o.strip() for o in origins_str.split(',') if o.strip()]

print(f"FastAPI CORS configured with ALLOWED_ORIGINS: {origins}") # For debugging

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
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
