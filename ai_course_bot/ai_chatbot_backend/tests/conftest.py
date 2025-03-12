import os
os.environ["ENVIRONMENT"] = "test"
# Supported LLM_MODE: local, mock, remote
os.environ["LLM_MODE"] = "mock"

import pytest
from app.api.deps import get_current_user
# from app.core.actions.llama_seletor import embedding_model
from fastapi.testclient import TestClient
from main import app


def dummy_get_current_user():
    return {"user_id": "test_user", "email": "test@example.com", "name": "Test User"}


@pytest.fixture(scope="session")
def client_unit():
    # Override dependencies for all tests.
    app.dependency_overrides[get_current_user] = dummy_get_current_user
    # app.dependency_overrides[get_model_pipeline] = dummy_pipeline
    os.environ["EMBEDDING_DIR"] = embedding_folder_for_test()

    client = TestClient(app)
    yield client
    # Clean up overrides after tests.
    app.dependency_overrides.clear()


# TODO: Implement integration test architecture in the future
# @pytest.fixture(scope="session")
# def client_integration():
#     os.environ["LLM_MODE"] = "remote"
#     app.dependency_overrides[get_current_user] = dummy_get_current_user
#     client = TestClient(app)
#     yield client
#     app.dependency_overrides.clear()


def embedding_folder_for_test():
    return os.path.join(os.path.dirname(__file__), "data", "embeddings")


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
    return """What is UC Berkeley TAI (Teaching Assistant Intelligence) project about?"""


@pytest.fixture
def trivia_question():
    return "Hello"
