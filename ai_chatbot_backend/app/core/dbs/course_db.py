from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
import os

from app.config import settings

"""
    TODO: merge practice db and course db together
"""
# Database configuration - can be overridden by environment variables
DATABASE_URL = os.getenv("DATABASE_URL", "sqlite:///./db/courses.db")

# Use SQLite for simplicity, can be changed to other databases as needed
SQLALCHEMY_DATABASE_URL = "sqlite:///./db/courses.db"

engine = create_engine(
    SQLALCHEMY_DATABASE_URL, connect_args={"check_same_thread": False}
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