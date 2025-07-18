from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker

# Separate database for metadata (files and problems)
METADATA_DATABASE_URL = "sqlite:///./db/metadata.db"

metadata_engine = create_engine(
    METADATA_DATABASE_URL, connect_args={"check_same_thread": False}
)
MetadataSessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=metadata_engine)

MetadataBase = declarative_base()


# Dependency to get metadata DB session
def get_metadata_db():
    db = MetadataSessionLocal()
    try:
        yield db
    finally:
        db.close()