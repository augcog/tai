from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker

# Separate database for metadata (files and problems)
CONTENT_DATABASE_URL = "sqlite:///./courses/content.db"

content_engine = create_engine(
    CONTENT_DATABASE_URL, connect_args={"check_same_thread": False}
)
ContentSessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=content_engine)

ContentBase = declarative_base()


# Dependency to get metadata DB session
def get_content_db():
    db = ContentSessionLocal()
    try:
        yield db
    finally:
        db.close()