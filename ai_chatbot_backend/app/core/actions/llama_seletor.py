from threading import Lock

from FlagEmbedding import BGEM3FlagModel
from dotenv import load_dotenv
from pydantic import BaseModel

# Global configuration for SQL database usage and extension paths.
SQLDB: bool = False
# TODO: Revise path design / configuration in the future for best practice.
EXT_VECTOR_PATH: str = (
    "ai_course_bot/ai-chatbot-backend/app/core/actions/dist/debug/vector0"
)
EXT_VSS_PATH: str = "ai_course_bot/ai-chatbot-backend/app/core/actions/dist/debug/vss0"


# Define a Pydantic Message model.
class Message(BaseModel):
    role: str
    content: str


# Load environment variables.
load_dotenv()

# Load the embedding model.
embedding_model = BGEM3FlagModel("BAAI/bge-m3", use_fp16=True)

# A threading lock for concurrency around local generation.
lock = Lock()
