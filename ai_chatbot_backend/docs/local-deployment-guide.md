# Local Backend Deployment Guide

## Prerequisites

- **Python** 3.10+
- **Poetry** 1.4+ ([install guide](https://python-poetry.org/docs/#installation))
- **Make** (pre-installed on macOS/Linux)

## Step 1: Download Required Files

Download the following two files:

| File | Description | Download Link |
|------|-------------|---------------|
| `db.zip` | SQLite databases (`courses.db`, `metadata.db`) | `<TODO: add link>` |
| `data.zip` | Course data files | `<TODO: add link>` |

## Step 2: Unzip and Place Files

Unzip the downloaded files and place them in the correct directories under `ai_chatbot_backend/`:

```bash
cd tai/ai_chatbot_backend

# Unzip database files into the db/ directory
unzip db.zip -d db/

# Unzip data files into the data/ directory
unzip data.zip -d data/
```

After this step, the directory structure should look like:

```
ai_chatbot_backend/
├── db/
│   ├── courses.db
│   └── metadata.db
├── data/
│   └── CS 61A/
│       └── ...
└── ...
```

## Step 3: Create the `.env` File

Copy the example environment file and edit it:

```bash
cp .env.example .env
```

Open `.env` and update the following fields:

```bash
# Set LLM mode to use OpenAI
llm_mode=openai

# Your OpenAI API Key
OPENAI_API_KEY=sk-your-actual-api-key-here

# OpenAI model
OPENAI_MODEL=gpt-4o

# Server URL (localhost)
SERVER_URL=http://127.0.0.1:8000
```

Leave the other settings at their default values unless you have specific requirements.

## Step 4: Install Dependencies and Run

```bash
# Install all dependencies (creates a local .venv)
make install

# Start the development server
make dev
```

The server will start at **http://localhost:8000**.

- API Docs: http://localhost:8000/docs
- Admin Panel: http://localhost:8000/admin

## Verification

Once the server is running, verify the setup:

```bash
# Check server health
curl http://localhost:8000/health

# Check database status
curl http://localhost:8000/database-status
```

## Troubleshooting

| Issue | Solution |
|-------|----------|
| `poetry: command not found` | Install Poetry: `curl -sSL https://install.python-poetry.org \| python3 -` |
| Installation fails on macOS | Make sure to use `make install` (not `make install-gpu`) |
| Database empty | Re-download and unzip `db.zip` into the `db/` directory |
| OpenAI API errors | Double-check `OPENAI_API_KEY` in your `.env` file |
