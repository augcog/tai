import paddle
from paddleocr import PaddleOCR

# Optionally force GPU or CPU usage:
# paddle.set_device('gpu')  # Uncomment if you have GPU and want to test it

print("Using device:", paddle.device.get_device())

