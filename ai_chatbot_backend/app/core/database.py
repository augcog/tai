from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
import os

from app.config import settings

"""
    TODO: merge practice db and course db together
"""
# Database configuration - can be overridden by environment variables
DATABASE_URL = os.getenv("DATABASE_URL", settings.DATABASE_URL)

# Use SQLite for simplicity, can be changed to other databases as needed
SQLALCHEMY_DATABASE_URL = settings.DATABASE_URL

engine = create_engine(
    SQLALCHEMY_DATABASE_URL, connect_args={"check_same_thread": False}
)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

Base = declarative_base()

# SQLite Practice Database
PRACTICE_DATABASE_URL = settings.PRACTICE_DATABASE_URL

practice_engine = create_engine(
    PRACTICE_DATABASE_URL, connect_args={"check_same_thread": False}
)

PracticeSessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=practice_engine)

PracticeBase = declarative_base()


# Dependency to get DB session
def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

# Dependency to get Practice DB session
def get_practice_db():
   db = PracticeSessionLocal()
   try:
       yield db
   finally:
        db.close()