# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## AI Guidance

- To save main context space, for code searches, inspections, troubleshooting or analysis, use code-searcher subagent where appropriate - giving the subagent full context background for the task(s) you assign it.
- For maximum efficiency, whenever you need to perform multiple independent operations, invoke all relevant tools simultaneously rather than sequentially.
- Before you finish, please verify your solution
- Do what has been asked; nothing more, nothing less.
- NEVER create files unless they're absolutely necessary for achieving your goal.
- ALWAYS prefer editing an existing file to creating a new one.
- NEVER proactively create documentation files (\*.md) or README files. Only create documentation files if explicitly requested by the User.
- When you update or modify memory bank files, also update related documentation
- **Session Accomplishment Tracking**: Only ask user for explicit approval to use the session-accomplishment-tracker agent when user has completed the full workflow: plan mode → approve plan → all TodoWrite tasks completed. In this specific scenario, ask: "Would you like me to use the session accomplishment tracker agent to document what we accomplished in this session?"

## Navigation & Component Routing

**TAI uses a modular monorepo structure. Use the appropriate specialized documentation based on your current working context:**

### Automatic Component Detection
When working with files in specific directories, consult the corresponding specialized documentation:

- **Working in `/ai_chatbot_backend/`** → Use `.claude/memory/claude-backend.md`
- **Working in `/rag/`** → Use `.claude/memory/claude-rag.md`
- **Working in `/evaluation/`** → Use `.claude/memory/claude-evaluation.md`

### Component-Specific Documentation
Existing specialized memory files located in `.claude/memory/`:
- **[Backend Documentation](.claude/memory/claude-backend.md)** - FastAPI service, database management, RAG integration
- **[RAG Documentation](.claude/memory/claude-rag.md)** - Document processing, embedding generation, file conversion
- **[Evaluation Documentation](.claude/memory/claude-evaluation.md)** - Dataset generation, performance testing, bias analysis

**Memory Bank System**: If additional Core Context Files are needed (CLAUDE-activeContext.md, CLAUDE-patterns.md, CLAUDE-decisions.md, CLAUDE-troubleshooting.md, CLAUDE-config-variables.md, CLAUDE-temp.md), generate them in `.claude/memory/` directory using the memory-bank-synchronizer agent.

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
8. **Session reports available**: Session accomplishment tracking available when appropriate (see AI Guidance for conditions)


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