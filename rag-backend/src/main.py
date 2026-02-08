import os
from dotenv import load_dotenv
from .core.logger import logger

# Load environment variables
load_dotenv()

from fastapi import FastAPI, HTTPException, Request, status
from fastapi.responses import JSONResponse
from fastapi.exceptions import RequestValidationError
from fastapi.middleware.cors import CORSMiddleware
from .api import query
from .core.exceptions import CustomException, http_exception_handler, custom_exception_handler, validation_exception_handler

app = FastAPI(
    title="RAG Chatbot Backend",
    description="API for the Physical AI & Humanoid Robotics textbook RAG chatbot.",
    version="1.0.0",
)

# --- CORS Update START ---
# Hum dono variables check kar rahe hain taake ghalti ki gunjayish na rahe
origins_str = os.getenv("ALLOWED_FRONTEND_ORIGINS") or os.getenv("ALLOWED_ORIGINS") or "https://speckit-hackaton.vercel.app"
origins = [o.strip() for o in origins_str.split(',') if o.strip()]

# Dashboard par nazar aayega ke backend ne kaunsa link uthaya
print(f"FASTAPI_CORS_ORIGINS_LOADED: {origins}") 

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
# --- CORS Update END ---

app.include_router(query.router, prefix="/api/v1")

# Register exception handlers
app.add_exception_handler(HTTPException, http_exception_handler)
app.add_exception_handler(CustomException, custom_exception_handler)
app.add_exception_handler(RequestValidationError, validation_exception_handler)
app.add_exception_handler(Exception, validation_exception_handler)

@app.get("/health", summary="Health Check")
def health_check():
    logger.info("Health check endpoint called.")
    return {"status": "ok"}