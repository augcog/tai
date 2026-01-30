#!/bin/bash
# vLLM Server Startup Script
# Starts all three vLLM servers in separate tmux windows with GPU memory monitoring

set -e

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_FILE="$SCRIPT_DIR/../.env"
SESSION_NAME="vllm-servers"
STARTUP_TIMEOUT=300  # 5 minutes per server

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Server configurations
CHAT_MODEL="cpatonn/Qwen3-30B-A3B-Thinking-2507-AWQ-4bit"
CHAT_PORT=8001
CHAT_GPUS="0,1"

EMBEDDING_MODEL="Qwen/Qwen3-Embedding-4B"
EMBEDDING_PORT=8002
EMBEDDING_GPUS="1"

WHISPER_MODEL="openai/whisper-large-v3"
WHISPER_PORT=8003
WHISPER_GPUS="0"

log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check prerequisites
check_prerequisites() {
    log_info "Checking prerequisites..."

    if ! command -v tmux &> /dev/null; then
        log_error "tmux is not installed. Install with: sudo apt install tmux"
        exit 1
    fi

    if ! command -v vllm &> /dev/null; then
        log_error "vllm is not installed. Install with: pip install vllm"
        exit 1
    fi

    if ! command -v nvidia-smi &> /dev/null; then
        log_error "nvidia-smi not found. NVIDIA drivers may not be installed."
        exit 1
    fi

    log_success "All prerequisites met"
}

# Load API key from .env file
load_api_key() {
    if [ -f "$ENV_FILE" ]; then
        VLLM_API_KEY=$(grep -E "^VLLM_API_KEY=" "$ENV_FILE" | cut -d'=' -f2 | tr -d '"' | tr -d "'")
        if [ -z "$VLLM_API_KEY" ]; then
            log_warning "VLLM_API_KEY not found in .env, using 'EMPTY'"
            VLLM_API_KEY="EMPTY"
        fi
    else
        log_warning ".env file not found at $ENV_FILE, using 'EMPTY' for API key"
        VLLM_API_KEY="EMPTY"
    fi
    log_info "Using API key: ${VLLM_API_KEY:0:10}..."
}

# Kill existing session if it exists
cleanup_existing_session() {
    if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
        log_warning "Existing tmux session '$SESSION_NAME' found. Killing it..."
        tmux kill-session -t "$SESSION_NAME"
    fi
}

# Wait for server to be ready by checking if port is listening
wait_for_server() {
    local port=$1
    local name=$2
    local timeout=$STARTUP_TIMEOUT
    local start_time=$(date +%s)

    log_info "Waiting for $name server on port $port..."

    while true; do
        local current_time=$(date +%s)
        local elapsed=$((current_time - start_time))

        if [ $elapsed -ge $timeout ]; then
            log_error "$name server failed to start within ${timeout}s"
            return 1
        fi

        # Check if port is listening
        if ss -tuln | grep -q ":$port "; then
            # Additional check: verify the API responds
            if curl -s "http://localhost:$port/v1/models" -H "Authorization: Bearer $VLLM_API_KEY" > /dev/null 2>&1; then
                log_success "$name server is ready on port $port (took ${elapsed}s)"
                return 0
            fi
        fi

        # Show progress every 30 seconds
        if [ $((elapsed % 30)) -eq 0 ] && [ $elapsed -gt 0 ]; then
            log_info "Still waiting for $name server... (${elapsed}s elapsed)"
        fi

        sleep 5
    done
}

# Start a vLLM server in a tmux window
start_server() {
    local window_name=$1
    local model=$2
    local port=$3
    local gpus=$4
    shift 4
    local extra_args="$@"

    log_info "Starting $window_name server..."
    log_info "  Model: $model"
    log_info "  Port: $port"
    log_info "  GPUs: $gpus"

    # Build the vllm command
    local cmd="CUDA_VISIBLE_DEVICES=$gpus vllm serve $model --port $port --api-key $VLLM_API_KEY $extra_args"

    # Create new window and run command
    tmux new-window -t "$SESSION_NAME" -n "$window_name"
    tmux send-keys -t "$SESSION_NAME:$window_name" "$cmd" C-m
}

# Print GPU memory usage report
print_gpu_report() {
    echo ""
    echo "=============================================="
    echo "         GPU MEMORY USAGE REPORT"
    echo "=============================================="
    nvidia-smi --query-gpu=index,name,memory.used,memory.total,utilization.gpu --format=csv,noheader,nounits | while IFS=',' read -r idx name used total util; do
        used_gb=$(echo "scale=2; $used / 1024" | bc)
        total_gb=$(echo "scale=2; $total / 1024" | bc)
        percent=$(echo "scale=1; $used * 100 / $total" | bc)
        echo "GPU $idx ($name):"
        echo "  Memory: ${used_gb}GB / ${total_gb}GB (${percent}%)"
        echo "  Utilization: ${util}%"
    done
    echo "=============================================="
    echo ""
}

# Main execution
main() {
    echo ""
    echo "=========================================="
    echo "     vLLM Server Startup Script"
    echo "=========================================="
    echo ""

    check_prerequisites
    load_api_key
    cleanup_existing_session

    # Create initial tmux session with a placeholder window
    log_info "Creating tmux session '$SESSION_NAME'..."
    tmux new-session -d -s "$SESSION_NAME" -n "startup"
    tmux send-keys -t "$SESSION_NAME:startup" "echo 'vLLM Servers Starting...'" C-m

    # Start Chat Model Server (Port 8001) - requires 2 GPUs for tensor parallel
    start_server "chat" "$CHAT_MODEL" "$CHAT_PORT" "$CHAT_GPUS" \
        "--tensor-parallel-size 2" \
        "--gpu-memory-utilization 0.47" \
        "--max-model-len 10000" \
        "--max_num_seqs 32" \
        "--reasoning-parser deepseek_r1"

    if ! wait_for_server $CHAT_PORT "Chat"; then
        log_error "Chat server failed to start. Check tmux session for errors."
        echo "To view logs: tmux attach -t $SESSION_NAME"
        exit 1
    fi

    # Start Embedding Server (Port 8002)
    start_server "embed" "$EMBEDDING_MODEL" "$EMBEDDING_PORT" "$EMBEDDING_GPUS" \
        "--max-model-len 10000" \
        "--max-num-seqs 32" \
        "--gpu-memory-utilization 0.4"

    if ! wait_for_server $EMBEDDING_PORT "Embedding"; then
        log_error "Embedding server failed to start. Check tmux session for errors."
        echo "To view logs: tmux attach -t $SESSION_NAME"
        exit 1
    fi

    # Start Whisper Server (Port 8003)
    start_server "whisper" "$WHISPER_MODEL" "$WHISPER_PORT" "$WHISPER_GPUS" \
        "--gpu-memory-utilization 0.37"

    if ! wait_for_server $WHISPER_PORT "Whisper"; then
        log_error "Whisper server failed to start. Check tmux session for errors."
        echo "To view logs: tmux attach -t $SESSION_NAME"
        exit 1
    fi

    # Kill the startup placeholder window
    tmux kill-window -t "$SESSION_NAME:startup" 2>/dev/null || true

    # Print GPU usage report
    print_gpu_report

    # Print success message and instructions
    echo ""
    log_success "All vLLM servers started successfully!"
    echo ""
    echo "Server Status:"
    echo "  - Chat Model:  http://localhost:$CHAT_PORT/v1  (GPUs: $CHAT_GPUS)"
    echo "  - Embedding:   http://localhost:$EMBEDDING_PORT/v1  (GPU: $EMBEDDING_GPUS)"
    echo "  - Whisper:     http://localhost:$WHISPER_PORT/v1  (GPU: $WHISPER_GPUS)"
    echo ""
    echo "tmux Controls:"
    echo "  Attach to session:  tmux attach -t $SESSION_NAME"
    echo "  Switch windows:     Ctrl+b then 0, 1, or 2"
    echo "  Stop a server:      Ctrl+C in that window"
    echo "  Detach:             Ctrl+b then d"
    echo ""
    echo "Test endpoints:"
    echo "  curl http://localhost:8001/v1/models -H 'Authorization: Bearer $VLLM_API_KEY'"
    echo "  curl http://localhost:8002/v1/models -H 'Authorization: Bearer $VLLM_API_KEY'"
    echo "  curl http://localhost:8003/v1/models -H 'Authorization: Bearer $VLLM_API_KEY'"
    echo ""
}

main "$@"
