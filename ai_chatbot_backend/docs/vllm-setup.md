# vLLM Model Services

This file documents how to start the vLLM servers that provide OpenAI-compatible APIs for the TAI backend.

## Prerequisites

- NVIDIA GPU(s) with sufficient VRAM
- vLLM installed (`pip install vllm`)
- CUDA properly configured

## Generating an API Key

To secure your vLLM servers, generate a random API key:

```bash
# Using Python
python -c "import secrets; print(secrets.token_urlsafe(32))"

# Using OpenSSL
openssl rand -base64 32

# Example output: dGhpc2lzYXNlY3VyZWFwaWtleWZvcnZsbG0K
```

Store the generated key securely and use the same key for:
1. The `--api-key` flag when starting vLLM servers
2. The `VLLM_API_KEY` environment variable in the backend's `.env` file

For development/testing, you can use `--api-key EMPTY` to disable authentication.

## Starting the Services

Replace `$VLLM_API_KEY` with your generated API key.

### Chat Model Server (Port 8001)

Main LLM for chat completions with reasoning support:

```bash
vllm serve cpatonn/Qwen3-30B-A3B-Thinking-2507-AWQ-4bit \
    --tensor-parallel-size 2 \
    --gpu-memory-utilization 0.47 \
    --max-model-len 10000 \
    --max_num_seqs 32 \
    --reasoning-parser deepseek_r1 \
    --port 8001 \
    --api-key $VLLM_API_KEY
```

### Embedding Server (Port 8002)

For RAG document retrieval:

```bash
vllm serve Qwen/Qwen3-Embedding-4B \
    --max-model-len 10000 \
    --port 8002 \
    --max-num-seqs 32 \
    --gpu-memory-utilization 0.4 \
    --api-key $VLLM_API_KEY
```

### Whisper Server (Port 8003)

For speech-to-text transcription:

```bash
vllm serve openai/whisper-large-v3 \
    --port 8003 \
    --gpu-memory-utilization 0.37 \
    --api-key $VLLM_API_KEY
```

## Configuration

Set the corresponding environment variables in the backend's `.env` file.

**Default (vLLM servers on same machine):**

```bash
VLLM_CHAT_URL=http://localhost:8001/v1
VLLM_EMBEDDING_URL=http://localhost:8002/v1
VLLM_WHISPER_URL=http://localhost:8003/v1
VLLM_API_KEY=EMPTY
```

**For remote vLLM servers:** Replace `localhost` with the GPU server's IP address.

## Running on Different Machines

When running vLLM servers on a different machine from the backend:

1. Generate an API key using the method above
2. Start vLLM servers on the GPU machine with `--host 0.0.0.0` to accept external connections
3. Configure the backend's `.env` with the GPU machine's IP addresses and API key
4. Ensure firewall allows connections on ports 8001-8003

Example for GPU server at `192.168.1.100`:

```bash
# On the GPU server
export VLLM_API_KEY="your-generated-api-key"
vllm serve cpatonn/Qwen3-30B-A3B-Thinking-2507-AWQ-4bit \
    --host 0.0.0.0 \
    --port 8001 \
    --api-key $VLLM_API_KEY \
    ... # other flags
```

```bash
# In backend .env file
VLLM_CHAT_URL=http://192.168.1.100:8001/v1
VLLM_EMBEDDING_URL=http://192.168.1.100:8002/v1
VLLM_WHISPER_URL=http://192.168.1.100:8003/v1
VLLM_API_KEY=your-generated-api-key
```
