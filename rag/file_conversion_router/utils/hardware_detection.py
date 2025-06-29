import platform
import subprocess

import torch


def detect_gpu_setup():
    if torch.cuda.is_available():
        return "cuda", torch.cuda.device_count()
    elif torch.backends.mps.is_available():  # For Macs with Apple Silicon
        return "mps", 1
    else:
        return "cpu", 0


def detect_is_apple_silicon():
    system = platform.system()

    if system == "Darwin":  # macOS
        # Check if it's Apple Silicon
        try:
            output = (
                subprocess.check_output(["sysctl", "-n", "machdep.cpu.brand_string"])
                .decode()
                .strip()
            )
            if "Apple" in output:
                return True
        except subprocess.CalledProcessError:
            pass

    # For all other cases (Intel, NVIDIA, others, or if Apple Silicon check fails)
    return False
