import requests
import json
from concurrent.futures import ThreadPoolExecutor, as_completed

API_URL = "http://128.32.43.233:8000/api/chat/completions"
HEADERS = {"Content-Type": "application/json"}
DATA = {
    "messages": [
        {
            "role": "user",
            "content": "what is this course about"
        }
    ],
    "temperature": 0.7,
    "max_tokens": 150,
    "stream": True,
    "rag": True,
    "course": "CS 61A"
}


def send_request():
    try:
        response = requests.post(
            API_URL, headers=HEADERS, data=json.dumps(DATA), timeout=30)
        # Return status and first 100 chars of response
        return response.status_code, response.text[:100]
    except Exception as e:
        return None, str(e)


def main():
    num_requests = int(
        input("How many requests do you want to send at the same time? "))
    results = []
    with ThreadPoolExecutor(max_workers=num_requests) as executor:
        futures = [executor.submit(send_request) for _ in range(num_requests)]
        for future in as_completed(futures):
            status, content = future.result()
            print(f"Status: {status}, Response: {content}")


if __name__ == "__main__":
    main()
