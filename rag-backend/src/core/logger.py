from loguru import logger

logger.add(
    "file.log", 
    rotation="10 MB", 
    compression="zip", 
    level="INFO", 
    format="{time} {level} {message}", 
    serialize=True
)
