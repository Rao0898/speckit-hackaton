from src.core.database import SessionLocal
from sqlalchemy import text  # << add this

db = SessionLocal()

# Delete all content_chunks
db.execute(text("DELETE FROM content_chunks"))  # << wrap with text()
db.commit()
db.close()

print("All content chunks deleted from DB")
