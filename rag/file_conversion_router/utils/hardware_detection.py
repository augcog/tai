import torch


def detect_gpu_setup():
    if torch.cuda.is_available():
        return "cuda", torch.cuda.device_count()
    elif torch.backends.mps.is_available():  # For Macs with Apple Silicon
        return "mps", 1
    else:
        return "cpu", 0
