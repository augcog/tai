import requests
import json
import time
import statistics
import os
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime

try:
    from dotenv import load_dotenv

    load_dotenv()
except ImportError:
    pass

API_URL = f"{os.getenv('SERVER_URL', 'http://127.0.0.1:8000')}/api/chat/completions"

api_auth_token = os.getenv("api_auth_token")
if not api_auth_token:
    raise ValueError(
        "api_auth_token environment variable is required. Please set it in your .env file or environment."
    )

HEADERS = {
    "Content-Type": "application/json",
    "Authorization": f"Bearer {api_auth_token}",
}
DATA = {
    "messages": [{"role": "user", "content": "what is this course about"}],
    "temperature": 0.7,
    "max_tokens": 150,
    "stream": True,
    "rag": True,
    "course": "CS61A",
}


def count_words(text):
    """Count words in text, handling streaming responses"""
    return len(text.split())


def send_request(request_id):
    """Send request and measure performance metrics"""
    try:
        start_time = time.time()
        request_sent_time = datetime.now()

        response = requests.post(
            API_URL, headers=HEADERS, data=json.dumps(DATA), timeout=30, stream=True
        )

        # Measure time to first byte/chunk
        first_chunk_time = None
        full_response = ""
        chunk_count = 0

        for chunk in response.iter_content(chunk_size=1024, decode_unicode=True):
            if chunk and first_chunk_time is None:
                first_chunk_time = time.time()
            if chunk:
                full_response += chunk
                chunk_count += 1

        end_time = time.time()

        # Calculate metrics
        total_time = end_time - start_time
        time_to_first_chunk = (
            first_chunk_time - start_time if first_chunk_time else None
        )
        word_count = count_words(full_response)
        words_per_second = word_count / total_time if total_time > 0 else 0
        response_size = len(full_response.encode("utf-8"))  # Size in bytes

        return {
            "request_id": request_id,
            "status_code": response.status_code,
            "success": True,
            "total_time": round(total_time, 3),
            "time_to_first_chunk": (
                round(time_to_first_chunk, 3) if time_to_first_chunk else None
            ),
            "word_count": word_count,
            "words_per_second": round(words_per_second, 2),
            "response_size_bytes": response_size,
            "chunk_count": chunk_count,
            "throughput_bytes_per_sec": (
                round(response_size / total_time, 2) if total_time > 0 else 0
            ),
            "response_preview": full_response[:100],
            "request_time": request_sent_time.strftime("%H:%M:%S.%f")[:-3],
        }

    except Exception as e:
        end_time = time.time()
        return {
            "request_id": request_id,
            "status_code": None,
            "success": False,
            "error": str(e),
            "total_time": (
                round(end_time - start_time, 3) if "start_time" in locals() else 0
            ),
            "request_time": (
                request_sent_time.strftime("%H:%M:%S.%f")[:-3]
                if "request_sent_time" in locals()
                else None
            ),
        }


def print_results(results):
    """Print individual results and aggregate statistics"""
    print("\n" + "=" * 80)
    print("INDIVIDUAL REQUEST RESULTS")
    print("=" * 80)

    successful_results = []

    for result in results:
        if result["success"]:
            print(f"Request #{result['request_id']} - Status: {result['status_code']}")
            print(
                f"  Time: {result['request_time']} | Total: {result['total_time']}s | TTFC: {result['time_to_first_chunk']}s"
            )
            print(
                f"  Words: {result['word_count']} | WPS: {result['words_per_second']} | Size: {result['response_size_bytes']} bytes"
            )
            print(
                f"  Throughput: {result['throughput_bytes_per_sec']} B/s | Chunks: {result['chunk_count']}"
            )
            print(f"  Preview: {result['response_preview']}")
            successful_results.append(result)
        else:
            print(f"Request #{result['request_id']} - FAILED")
            print(f"  Error: {result['error']}")
            print(
                f"  Time: {result['request_time']} | Duration: {result['total_time']}s"
            )
        print("-" * 80)

    if successful_results:
        print("\nAGGREGATE PERFORMANCE STATISTICS")
        print("=" * 80)

        # Calculate aggregate metrics
        total_times = [r["total_time"] for r in successful_results]
        ttfc_times = [
            r["time_to_first_chunk"]
            for r in successful_results
            if r["time_to_first_chunk"]
        ]
        words_per_sec = [r["words_per_second"] for r in successful_results]
        word_counts = [r["word_count"] for r in successful_results]
        throughputs = [r["throughput_bytes_per_sec"] for r in successful_results]

        print(f"Successful Requests: {len(successful_results)}/{len(results)}")
        print(f"Success Rate: {len(successful_results) / len(results) * 100:.1f}%")
        print()

        print("RESPONSE TIME STATISTICS:")
        print(f"  Average: {statistics.mean(total_times):.3f}s")
        print(f"  Median: {statistics.median(total_times):.3f}s")
        print(f"  Min: {min(total_times):.3f}s")
        print(f"  Max: {max(total_times):.3f}s")
        print(
            f"  Std Dev: {statistics.stdev(total_times):.3f}s"
            if len(total_times) > 1
            else "  Std Dev: N/A"
        )
        print()

        if ttfc_times:
            print("TIME TO FIRST CHUNK STATISTICS:")
            print(f"  Average: {statistics.mean(ttfc_times):.3f}s")
            print(f"  Median: {statistics.median(ttfc_times):.3f}s")
            print(f"  Min: {min(ttfc_times):.3f}s")
            print(f"  Max: {max(ttfc_times):.3f}s")
            print()

        print("WORDS PER SECOND STATISTICS:")
        print(f"  Average: {statistics.mean(words_per_sec):.2f} WPS")
        print(f"  Median: {statistics.median(words_per_sec):.2f} WPS")
        print(f"  Min: {min(words_per_sec):.2f} WPS")
        print(f"  Max: {max(words_per_sec):.2f} WPS")
        print()

        print("CONTENT STATISTICS:")
        print(f"  Average Words per Response: {statistics.mean(word_counts):.1f}")
        print(f"  Total Words Generated: {sum(word_counts)}")
        print(f"  Average Throughput: {statistics.mean(throughputs):.2f} bytes/sec")
        print()

        print("CONCURRENCY PERFORMANCE:")
        total_test_time = max(total_times)  # Approximate since concurrent
        total_words = sum(word_counts)
        aggregate_wps = total_words / total_test_time if total_test_time > 0 else 0
        print(f"  Aggregate Words/Second: {aggregate_wps:.2f} WPS")
        print(
            f"  Requests per Second: {len(successful_results) / total_test_time:.2f} RPS"
        )


def main():
    num_requests = int(
        input("How many requests do you want to send at the same time? ")
    )

    print(f"\nSending {num_requests} concurrent requests to {API_URL}")
    print("Starting concurrent test...")

    start_time = time.time()
    results = []

    with ThreadPoolExecutor(max_workers=num_requests) as executor:
        futures = [executor.submit(send_request, i + 1) for i in range(num_requests)]

        for future in as_completed(futures):
            result = future.result()
            results.append(result)
            # Show progress
            if result["success"]:
                print(
                    f"✓ Request #{result['request_id']} completed in {result['total_time']}s"
                )
            else:
                print(
                    f"✗ Request #{result['request_id']} failed: {result.get('error', 'Unknown error')}"
                )

    total_test_time = time.time() - start_time
    print(f"\nAll requests completed in {total_test_time:.3f}s")

    # Sort results by request ID for consistent display
    results.sort(key=lambda x: x["request_id"])

    print_results(results)


if __name__ == "__main__":
    main()
