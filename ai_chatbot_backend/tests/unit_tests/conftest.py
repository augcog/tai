import os

os.environ["ENVIRONMENT"] = "test"
# Supported LLM_MODE: local, mock, remote
os.environ["LLM_MODE"] = "mock"

import pytest
from fastapi.testclient import TestClient
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from sqlalchemy.pool import StaticPool

from app.core.dbs.course_db import get_db
from app.core.models.courses import Base
from app.api.deps import get_current_user
from main import app

# Create an in-memory SQLite database
SQLALCHEMY_DATABASE_URL = "sqlite:///:memory:"
engine = create_engine(
    SQLALCHEMY_DATABASE_URL,
    connect_args={"check_same_thread": False},
    poolclass=StaticPool,
)
TestingSessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)


def dummy_get_current_user():
    """Dummy user for testing."""
    return {"user_id": "test_user", "email": "test@example.com", "name": "Test User"}


@pytest.fixture
def db_session():
    """Create a fresh database session for each test."""
    Base.metadata.create_all(bind=engine)
    session = TestingSessionLocal()
    try:
        yield session
    finally:
        session.close()
        Base.metadata.drop_all(bind=engine)



@pytest.fixture
def client_unit(db_session):
    """Create a test client with database session and authentication."""
    os.environ["EMBEDDING_DIR"] = embedding_folder_for_test()

    def override_get_db():
        try:
            yield db_session
        finally:
            pass

    app.dependency_overrides[get_db] = override_get_db
    app.dependency_overrides[get_current_user] = dummy_get_current_user
    return TestClient(app)


def embedding_folder_for_test():
    # the abs path of this file
    return os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "data", "embeddings"
    )


@pytest.fixture
def cs61a_question():
    return """Here's a concise CS61A-style benchmark question:

def foo(n):
    def bar(m):
        nonlocal n
        n += 1
        return n + m
    return bar

f = foo(10)
print(f(3), f(4), f(5))

Question:
1. What is printed when the final print statement executes?
2. Briefly explain why that output occurs (i.e., how the environment/`nonlocal` assignment affects `n` in each call).
"""


@pytest.fixture
def cs61a_quick_question():
    return """What is recursion about in CS 61A?"""


@pytest.fixture
def tai_trivia_question():
    return (
        """What is UC Berkeley TAI (Teaching Assistant Intelligence) project about?"""
    )


@pytest.fixture
def trivia_question():
    return "Hello"
