vllm serve cpatonn/Qwen3-30B-A3B-Thinking-2507-AWQ-4bit --tensor-parallel-size 2 --gpu-memory-utilization 0.47 --max-model-len 10000 --max_num_seqs 32 --reasoning-parser deepseek_r1 --port 8001 --api-key UJnYZomLkoSUUFqvxU30ULBaGISzPfz3

vllm serve openai/whisper-large-v3 --port 8003 --gpu-memory-utilization 0.37 --api-key UJnYZomLkoSUUFqvxU30ULBaGISzPfz3

vllm serve Qwen/Qwen3-Embedding-4B --max-model-len 10000 --port 8002 --max-num-seqs 32 --gpu-memory-utilization 0.4 --api-key UJnYZomLkoSUUFqvxU30ULBaGISzPfz3