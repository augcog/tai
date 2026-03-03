from transformers import AutoTokenizer
tok = AutoTokenizer.from_pretrained("cpatonn/Qwen3-30B-A3B-Thinking-2507-AWQ-4bit")
print(tok.special_tokens_map)
print(tok.tokenize("Let's think step by step.<|thinking|>"))