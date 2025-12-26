import sys
import os
from pathlib import Path

# Add the project root to the Python path
project_root = Path(__file__).parent
sys.path.insert(0, str(project_root))

from src.scripts.ingest import ingest_content
from src.models import db
from src.core.database import engine

if __name__ == "__main__":
    # Create tables if they don't exist
    db.Base.metadata.create_all(bind=engine)
    ingest_content()