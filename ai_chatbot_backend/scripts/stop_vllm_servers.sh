#!/bin/bash
# vLLM Server Shutdown Script
# Stops all vLLM servers by killing the tmux session

SESSION_NAME="vllm-servers"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo ""
echo "=========================================="
echo "     vLLM Server Shutdown Script"
echo "=========================================="
echo ""

# Check if session exists
if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
    echo -e "${YELLOW}[INFO]${NC} Killing tmux session '$SESSION_NAME'..."
    tmux kill-session -t "$SESSION_NAME"
    echo -e "${GREEN}[SUCCESS]${NC} All vLLM servers stopped."
    echo ""

    # Show GPU memory after shutdown
    echo "GPU Memory Status:"
    nvidia-smi --query-gpu=index,memory.used,memory.total --format=csv,noheader,nounits | while IFS=',' read -r idx used total; do
        echo "  GPU $idx: ${used}MB / ${total}MB"
    done
    echo ""
else
    echo -e "${RED}[ERROR]${NC} No tmux session '$SESSION_NAME' found."
    echo ""
    echo "Check if vLLM servers are running with:"
    echo "  tmux ls"
    echo "  ps aux | grep vllm"
    echo ""
fi
