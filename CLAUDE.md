# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Navigation & Component Routing

**TAI uses a modular monorepo structure. Use the appropriate specialized documentation based on your current working context:**

### Automatic Component Detection
When working with files in specific directories, consult the corresponding specialized documentation:

- **Working in `/ai_chatbot_backend/`** → Use `.claude/memory/claude-backend.md`
- **Working in `/rag/`** → Use `.claude/memory/claude-rag.md`
- **Working in `/evaluation/`** → Use `.claude/memory/claude-evaluation.md`

### Component-Specific Documentation
- **[Backend Documentation](.claude/memory/claude-backend.md)** - FastAPI service, database management, RAG integration
- **[RAG Documentation](.claude/memory/claude-rag.md)** - Document processing, embedding generation, file conversion
- **[Evaluation Documentation](.claude/memory/claude-evaluation.md)** - Dataset generation, performance testing, bias analysis

### Specialized Agents
- **[Session Accomplishment Tracker](.claude/agents/session-accomplishment-tracker.md)** - Documents session outcomes, progress, and technical decisions
- **[Memory Bank Synchronizer](.claude/agents/memory-bank-synchronizer.md)** - Maintains documentation consistency with codebase
- **[Code Searcher](.claude/agents/code-searcher.md)** - Advanced code exploration and pattern analysis

**Always consult the component-specific documentation first for detailed guidance, then refer back to this file for cross-component interactions and overall architecture.**

## Project Overview

TAI (Teaching Assistant Intelligence) is a comprehensive AI-powered educational platform with RAG (Retrieval-Augmented Generation) capabilities. The system consists of three main components:

- **AI Chatbot Backend**: FastAPI service with RAG capabilities, course management, and file services
- **RAG Pipeline**: Document processing, embedding generation, and file conversion system  
- **Evaluation Tools**: Dataset generation and testing framework

## Architecture

### Monorepo Structure
```
tai/
├── ai_chatbot_backend/     # FastAPI backend service
├── rag/                    # RAG pipeline and file processing
└── evaluation/             # Evaluation and dataset tools
```

### Component Architecture
- **Backend**: FastAPI + SQLAlchemy with hybrid SQLite/MongoDB storage
- **RAG Pipeline**: Multi-format document processing with BGE-M3 embeddings
- **AI Models**: Llama3 base model, BGE-M3 embeddings, OpenAI integration
- **Storage**: SQLite with vector extensions (sqlite-vss) + MongoDB cloud backup
- **Authentication**: JWT token-based with Google OAuth support

## Development Commands

### AI Chatbot Backend
Navigate to `cd ai_chatbot_backend` first:

```bash
# Setup and installation
make install              # Install dependencies in local .venv
make db-init              # Initialize database (loads from MongoDB if empty)

# Development
make dev                  # Start development server (http://localhost:8000)
make server              # Start production server

# Testing
make test                # Run all tests
make test-unit           # Run unit tests only
make test-integration    # Run integration tests only
make test-api           # Run API tests

# Code quality
make lint                # Run linting checks (ruff)
make format              # Format code and auto-fix issues

# Database management
make db-status           # Check database status
make admin               # Open admin interface (http://localhost:8000/admin)
```

### RAG Pipeline
Navigate to `cd rag` first:

```bash
# Installation options
make install-basic       # Core dependencies only
make install-cv          # Add computer vision support
make install-ocr         # Add OCR capabilities (large download)
make install-full        # All features (5GB+ download)

# Document processing  
make convert INPUT=docs/ OUTPUT=processed/    # Convert documents
make embed INPUT=processed/ OUTPUT=embeddings/  # Generate embeddings
make process INPUT=docs/ OUTPUT=final/        # Full pipeline

# Development
make dev                 # Start RAG development server
make test                # Run tests (excludes slow/GPU tests)
make lint                # Run linting checks
make format              # Format code
```

### Root Level Commands
```bash
# Workspace-wide operations (run from repository root)
poetry install           # Install all project dependencies
poetry run pytest       # Run tests across all projects
```

## Key Technical Concepts

### Database Architecture
The system uses a **hybrid database approach**:
- **Local SQLite**: `courses.db` and `metadata.db` for fast access
- **MongoDB Cloud**: Backup and synchronization with cloud storage
- **Auto-initialization**: Loads from MongoDB when local databases are empty

### RAG System
- **Embedding Model**: BAAI/bge-m3 (BGE-M3) with multi-modal scoring
- **Document Processing**: PDF, Markdown, HTML, Python, Jupyter notebooks
- **Vector Storage**: SQLite with vector extensions (sqlite-vss)
- **Chunking Strategies**: Recursive, paragraph, sentence, and semantic splitting

### AI Model Integration
- **Local Models**: Llama3 via VLLM for on-device inference
- **Remote APIs**: OpenAI, Anthropic integration
- **Model Selection**: Configurable via `LLM_MODE` environment variable

## Important File Locations

### Configuration Files
- `ai_chatbot_backend/.env` - Backend environment configuration
- `rag/.env` - RAG pipeline configuration  
- `ai_chatbot_backend/config/database_mapping.json` - Database mapping configuration

### Key Source Files
- `ai_chatbot_backend/main.py` - FastAPI application entry point
- `ai_chatbot_backend/app/api/router.py` - API route definitions
- `ai_chatbot_backend/app/services/` - Business logic and RAG services
- `rag/file_conversion_router/` - Document conversion services
- `rag/file_conversion_router/embedding/` - Embedding generation

### Database Scripts
- `ai_chatbot_backend/scripts/initialize_db_and_files.py` - Database initialization
- `ai_chatbot_backend/scripts/seed_mongodb.py` - MongoDB seeding
- `ai_chatbot_backend/scripts/export_sqlite_data.py` - SQLite data export

### Documentation and Session Tracking
- `.claude/memory/` - Component-specific technical documentation
- `.claude/agents/` - Specialized agent documentation and workflows
- `.claude/jobs/` - Session accomplishment reports and development history
- `.claude/commands/` - Reusable command templates and procedures

## Development Workflows

### New Feature Development
1. Navigate to appropriate component directory (`ai_chatbot_backend` or `rag`)
2. Create feature branch from `main`
3. Install dependencies: `make install` or `make install-basic`
4. Initialize databases (backend): `make db-init`
5. Start development server: `make dev`
6. Run tests: `make test`
7. Check code quality: `make lint` and `make format`
8. **Document session outcomes**: Use [session-accomplishment-tracker](.claude/agents/session-accomplishment-tracker.md) to create comprehensive session reports

### Session Documentation Workflow
1. **Session Start**: Begin with clear objectives, optionally use TodoWrite for complex tasks
2. **During Development**: Track progress and maintain TodoWrite task list
3. **Session End**: Generate accomplishment report using session-accomplishment-tracker agent
4. **Report Storage**: Session reports automatically saved to `.claude/jobs/` directory
5. **Follow-up Sessions**: Reference previous session reports when resuming related work

### TodoWrite Integration with Session Tracking

**When to Generate Session Reports:**
- **After TodoWrite Completion**: Always generate when all TodoWrite tasks are completed
- **Complex Multi-Step Tasks**: For sessions with 3+ distinct development steps
- **Technical Decision Sessions**: When significant architectural choices were made
- **Debugging Sessions**: When multiple problems were solved with reusable solutions
- **File-Heavy Sessions**: When multiple files were created/modified
- **Setup/Configuration Work**: When system or environment changes were made

**IMPORTANT - Automatic User Prompting:**
When a TodoWrite task list is fully completed (all tasks marked as "completed"), Claude should **automatically ask the user** if they would like to generate a session accomplishment report:

> *"We've successfully completed all the tasks in our todo list! Would you like me to create a session accomplishment report to document what we achieved? This will help track our progress and serve as a reference for future development work."*

**TodoWrite → Session Tracker Workflow:**
```
1. Create TodoWrite task list for complex work
2. Work through tasks, updating status to completed
3. When ALL tasks are completed → AUTOMATICALLY ask user about session report
4. If user agrees: Use session-accomplishment-tracker agent
5. Generate comprehensive report in .claude/jobs/
6. Report includes TodoWrite task completion summary and technical outcomes
```

### Database Operations
```bash
# Fresh setup from MongoDB
cd ai_chatbot_backend
make db-init

# Export local data to MongoDB
python scripts/export_sqlite_data.py
python scripts/seed_mongodb.py

# Disaster recovery
rm -f db/courses.db db/metadata.db
python scripts/initialize_db_and_files.py --force
```

### RAG Pipeline Development
```bash
# Process documents end-to-end
cd rag
make install-basic
make convert INPUT=your_docs/ OUTPUT=processed/
make embed INPUT=processed/ OUTPUT=embeddings/

# Add new document format support
# Extend rag/file_conversion_router/conversion/base_converter.py
# Add new converter in rag/file_conversion_router/conversion/
```

## Environment Setup

### Backend Environment Variables
```bash
# Required
DATA_DIR=/path/to/course/files
MONGODB_URI=mongodb+srv://...

# Optional
LLM_MODE=remote  # or local, openai, anthropic
HOST=localhost
PORT=8000
SECRET_KEY=your-secret-key
```

### RAG Environment Variables
```bash
# Document processing
INPUT_DIR=/path/to/documents
OUTPUT_DIR=/path/to/processed
EMBEDDING_MODEL=BAAI/bge-m3
DEVICE=cuda  # or cpu, mps
OCR_ENABLED=true
```

## Testing Strategy

### Backend Tests
- **Unit Tests**: Individual component testing
- **Integration Tests**: API endpoint testing with real database
- **API Tests**: Full request/response cycle testing
- **Test Fixtures**: Comprehensive request/response examples

### RAG Tests
- **Conversion Tests**: Document format conversion accuracy
- **Embedding Tests**: Embedding generation and quality
- **Service Tests**: External service integration
- **Performance Tests**: Marked with `@pytest.mark.slow`

### Running Tests
```bash
# Backend
cd ai_chatbot_backend && make test
pytest -m "unit" tests/                    # Unit tests only
pytest -m "integration" tests/             # Integration tests only

# RAG
cd rag && make test
pytest -m "not slow and not gpu" tests/    # Exclude resource-intensive tests
```

## Common Patterns

### Error Handling
- Use FastAPI's `HTTPException` for API errors
- Implement proper logging with structured formats
- Handle MongoDB connectivity gracefully with fallbacks

### Database Patterns
- Use SQLAlchemy ORM models in `app/core/models/`
- Implement service layer in `app/services/` for business logic
- Use Pydantic schemas in `app/schemas/` for request/response validation

### RAG Development Patterns
- Extend `BaseConverter` for new document formats
- Use `TaskManager` for distributed processing
- Implement caching in `utils/conversion_cache.py`

## Deployment Notes

### Production Configuration
```bash
# Backend production
export RELOAD=false
export LOG_LEVEL=info
export ENVIRONMENT=production

# RAG production  
export TORCH_THREADS=4
export OMP_NUM_THREADS=4
export CUDA_VISIBLE_DEVICES=0
```

### Docker Support
Both components include Docker support with production-ready configurations.

## Cross-Component Interactions

### Backend ↔ RAG Integration
- **Backend consumes RAG embeddings**: Backend loads embeddings generated by RAG pipeline
- **Shared embedding models**: Both components use BGE-M3 embeddings
- **Document processing workflow**: RAG processes documents → Backend serves them via API

### Evaluation ↔ System Testing
- **Backend evaluation**: Evaluation tests backend API responses and RAG accuracy
- **RAG performance testing**: Evaluation measures document processing quality
- **Dataset generation**: Evaluation creates test datasets for system validation

### Development Workflow Across Components
```bash
# 1. Process documents (RAG)
cd rag && make process INPUT=new_docs/ OUTPUT=processed/

# 2. Update backend with new embeddings (Backend)
cd ../ai_chatbot_backend && make db-init

# 3. Test system performance (Evaluation)
cd ../evaluation/dataset_generate && python -m evaluation.dataset_generate.generate test_data.json
```

## Troubleshooting

### Component-Specific Issues
Refer to the specialized documentation for detailed troubleshooting:
- **Backend Issues**: See `.claude/memory/claude-backend.md` → "Common Issues & Troubleshooting"
- **RAG Issues**: See `.claude/memory/claude-rag.md` → "Common Issues & Troubleshooting"  
- **Evaluation Issues**: See `.claude/memory/claude-evaluation.md` → "Common Issues & Troubleshooting"

### Cross-Component Issues
- **Embedding compatibility**: Ensure RAG and Backend use same BGE-M3 model version
- **Database sync**: Backend must reinitialize after RAG processes new documents
- **Environment conflicts**: Each component uses isolated virtual environments

### Debug Commands
```bash
# Check overall system health
cd ai_chatbot_backend && make health
cd ../rag && make info  
cd ../evaluation/dataset_generate && make test-no-api
```