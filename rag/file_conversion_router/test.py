import paddle
from paddleocr import PaddleOCR

# Optionally force GPU or CPU usage:
# paddle.set_device('gpu')  # Uncomment if you have GPU and want to test it

print("Using device:", paddle.device.get_device())

# Initialize PaddleOCR (ensure use_gpu is set as desired)
ocr = PaddleOCR(use_gpu=True)
result = ocr.ocr("path_to_a_sample_image.jpg", cls=True)
print(result)
