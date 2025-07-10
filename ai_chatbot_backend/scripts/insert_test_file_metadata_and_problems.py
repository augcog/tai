import uuid
from sqlalchemy.orm import Session
from app.core.database import SessionLocal
from app.core.models.problems import ProblemModel
from app.core.models.file_metadata import FileMetadataModel

def main():
    db: Session = SessionLocal()
    try:
        # 1. Create file_metadata
        file_metadata = FileMetadataModel(
            file_name="test_file.py",
            url="/static/test_file.py",
            sections=[]
        )
        db.add(file_metadata)
        db.commit()
        db.refresh(file_metadata)
        
        # 2. Generate two different problems, associated with file_metadata
        problem1 = ProblemModel(
            uuid=uuid.uuid4(),
            file_uuid=file_metadata.uuid,
            problem_index="1.1",
            problem_id="PROB001",
            problem_content="Basic geography question",
            question_id=1,
            question="What is the capital of France?",
            choices=["Berlin", "Paris", "London"],
            answer=1,
            explanation="Paris is the capital of France."
        )
        problem2 = ProblemModel(
            uuid=uuid.uuid4(),
            file_uuid=file_metadata.uuid,
            problem_index="1.2",
            problem_id="PROB002",
            problem_content="Basic arithmetic question",
            question_id=2,
            question="What is 2 + 2?",
            choices=["3", "4", "5"],
            answer=1,
            explanation="2 + 2 = 4."
        )

        # 3. Insert problems
        db.add(problem1)
        db.add(problem2)
        db.commit()
        
        print("Test data inserted successfully!")
        print(f"File Metadata UUID: {file_metadata.uuid}")
        print(f"Problem1 UUID: {problem1.uuid}")
        print(f"Problem2 UUID: {problem2.uuid}")
    finally:
        db.close()

if __name__ == "__main__":
    main() 