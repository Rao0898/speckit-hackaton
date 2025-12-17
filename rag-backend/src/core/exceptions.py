from fastapi import HTTPException, Request, status
from fastapi.responses import JSONResponse
from loguru import logger

class CustomException(HTTPException):
    def __init__(self, status_code: int, detail: str, name: str = "CustomException"):
        super().__init__(status_code=status_code, detail=detail)
        self.name = name

async def http_exception_handler(request: Request, exc: HTTPException):
    logger.error(f"HTTP Exception: {exc.status_code} - {exc.detail}")
    return JSONResponse(
        status_code=exc.status_code,
        content={"detail": exc.detail},
    )

async def custom_exception_handler(request: Request, exc: CustomException):
    logger.error(f"{exc.name} Exception: {exc.status_code} - {exc.detail}")
    return JSONResponse(
        status_code=exc.status_code,
        content={"detail": exc.detail, "name": exc.name},
    )

async def validation_exception_handler(request: Request, exc: Exception):
    # This handler captures Pydantic validation errors and other generic exceptions
    logger.error(f"Validation/Generic Exception: {exc}")
    return JSONResponse(
        status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
        content={"detail": str(exc)},
    )
