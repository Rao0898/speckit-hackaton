from sqlalchemy.orm import Session
from ..models import db

class ContentChunkDAO:
    def __init__(self, db_session: Session):
        self.db = db_session

    def get_chunk_by_hash(self, content_hash: str) -> db.ContentChunk:
        return self.db.query(db.ContentChunk).filter(db.ContentChunk.content_hash == content_hash).first()

    def create_chunk(self, chunk_data: dict) -> db.ContentChunk:
        chunk = db.ContentChunk(**chunk_data)
        self.db.add(chunk)
        self.db.commit()
        self.db.refresh(chunk)
        return chunk

    def bulk_create_chunks(self, chunks_data: list[dict]) -> list[db.ContentChunk]:
        chunks = [db.ContentChunk(**data) for data in chunks_data]
        self.db.add_all(chunks)
        self.db.commit()
        return chunks
