import uuid
from sqlalchemy import Column, String, Text, DateTime, func, TypeDecorator
import json
from ..core.database import Base

class JSONEncodedDict(TypeDecorator):
    """Enables JSON storage by encoding and decoding on the fly."""
    impl = String

    def process_bind_param(self, value, dialect):
        if value is not None:
            value = json.dumps(value)
        return value

    def process_result_value(self, value, dialect):
        if value is not None:
            value = json.loads(value)
        return value

class ContentChunk(Base):
    __tablename__ = "content_chunks"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    content_hash = Column(String, nullable=False, unique=True)
    source_file = Column(String, nullable=False)
    text = Column(Text, nullable=False)
    extra_data = Column(JSONEncodedDict)
    created_at = Column(DateTime, server_default=func.now())
    updated_at = Column(DateTime, server_default=func.now(), onupdate=func.now())
