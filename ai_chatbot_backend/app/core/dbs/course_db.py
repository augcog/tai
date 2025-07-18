from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker

"""
    TODO: merge practice db and course db together
"""
# Database configuration - use same database as metadata
COURSE_DATABASE_URL = "sqlite:///./db/metadata.db"

engine = create_engine(
    COURSE_DATABASE_URL, connect_args={"check_same_thread": False}
)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

Base = declarative_base()


# Dependency to get DB session
def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()