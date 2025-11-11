import time
import asyncio
import numpy as np
from concurrent.futures import ThreadPoolExecutor
from FlagEmbedding import BGEM3FlagModel
from typing import Dict, Any


class AsyncEmbeddingEngine:
    def __init__(self, model, max_workers: int = 3):
        self.model = model
        self.queue = asyncio.Queue()
        self.running = False
        self.max_workers = max_workers
        self.executor = ThreadPoolExecutor(max_workers=max_workers)
        self.active_batches = 0

    async def start(self):
        if not self.running:
            self.running = True
            self._worker_task = asyncio.create_task(self._worker())

    async def stop(self):
        self.running = False
        await self.queue.put((None, None))
        await self._worker_task
        self.executor.shutdown(wait=True)

    async def encode_async(self, text: str) -> Dict[str, Any]:
        loop = asyncio.get_running_loop()
        fut = loop.create_future()
        await self.queue.put((text, fut))
        return await fut

    async def _worker(self):
        while self.running:
            batch = []

            # Wait for first item
            try:
                item = await asyncio.wait_for(self.queue.get(), timeout=0.1)
                text, fut = item

                if text is None and fut is None:
                    break

                batch.append((text, fut))
            except asyncio.TimeoutError:
                continue

            # Collect all immediately available items
            while not self.queue.empty():
                try:
                    item = self.queue.get_nowait()
                    text, fut = item

                    if text is None and fut is None:
                        await self.queue.put((None, None))
                        break

                    batch.append((text, fut))
                except asyncio.QueueEmpty:
                    break

            if not batch:
                continue

            # Process in thread if resources available
            if self.active_batches < self.max_workers:
                asyncio.create_task(self._process_batch(batch))
            else:
                # Put items back and wait
                for item in batch:
                    await self.queue.put(item)
                await asyncio.sleep(0.1)

    async def _process_batch(self, batch):
        if not batch:
            return

        self.active_batches += 1

        try:
            texts, futures = zip(*batch)
            print(f"🔄 Worker {self.active_batches}: Processing {len(texts)} texts in parallel thread")

            loop = asyncio.get_running_loop()
            batch_results = await loop.run_in_executor(
                self.executor,
                self._encode_batch,
                texts
            )

            dense_vecs = batch_results.get('dense_vecs', [])
            sparse_vecs = batch_results.get('lexical_weights', [])
            colbert_vecs = batch_results.get('colbert_vecs', [])

            for i, fut in enumerate(futures):
                if not fut.cancelled():
                    result = {
                        'dense_vecs': dense_vecs[i],
                        'sparse_vecs': sparse_vecs[i],
                        'colbert_vecs': colbert_vecs[i],
                    }
                    fut.set_result(result)

        except Exception as e:
            for _, fut in batch:
                if not fut.cancelled():
                    fut.set_exception(e)
        finally:
            self.active_batches -= 1

    def _encode_batch(self, texts):
        """Encode batch using model (runs in ThreadPoolExecutor)"""
        return self.model.encode(
            texts,
            return_dense=True,
            return_sparse=True,
            return_colbert_vecs=True
        )


""" === Test functions for async embedding engine === """


def test_non_async_embedding(model, messages):
    """Test regular synchronous embedding (no batching)"""
    print(f"\n🐌 === NON-ASYNC EMBEDDING TEST ===")
    print(f"📝 Processing {len(messages)} requests one by one (no batching)")

    start_time = time.time()
    results = []

    for text in messages:
        result = model.encode(
            text,
            return_dense=True,
            return_sparse=True,
            return_colbert_vecs=True
        )
        results.append(result)

    end_time = time.time()
    total_time = end_time - start_time
    throughput = len(messages) / total_time

    print(f"  ⏱️  Total time: {total_time:.3f}s")
    print(f"  🚀 Throughput: {throughput:.1f} requests/second")
    return results, total_time


async def test_async_embedding(model, messages, max_workers=3):
    """Test async embedding with dynamic batching"""
    print(f"\n⚡ === ASYNC EMBEDDING TEST ===")
    print(f"📝 Processing {len(messages)} requests with dynamic batching")

    service = AsyncEmbeddingEngine(model, max_workers=max_workers)
    await service.start()

    start_time = time.time()
    tasks = []

    for text in messages:
        task = service.encode_async(text)
        tasks.append(task)

    results = await asyncio.gather(*tasks)
    end_time = time.time()

    await service.stop()

    total_time = end_time - start_time
    throughput = len(messages) / total_time

    print(f"  ⏱️  Total time: {total_time:.3f}s")
    print(f"  🚀 Throughput: {throughput:.1f} requests/second")
    return results, total_time


def verify_results_match(non_async_results, async_results, tolerance=1e-3):
    """Verify that async results match non-async results (ground truth)"""
    print(f"\n🔍 === VERIFYING RESULT ACCURACY ===")

    if len(non_async_results) != len(async_results):
        print(f"  ❌ Length mismatch: {len(non_async_results)} vs {len(async_results)}")
        return False

    mismatches = 0
    total_comparisons = 0

    for i, (non_async_result, async_result) in enumerate(zip(non_async_results, async_results)):
        # Compare each vector type
        for vector_type in ['dense_vecs', 'sparse_vecs', 'colbert_vecs']:
            if vector_type == 'sparse_vecs':
                non_async_sparse = non_async_result.get('lexical_weights', {})
                async_sparse = async_result.get('sparse_vecs', {})
                # for i in range(len(non_async_sparse)):
                #     print(f"{vector_type}: {i} difference between non-async and async: {non_async_sparse[i] - async_sparse[i]}")
                if set(non_async_sparse.keys()) != set(async_sparse.keys()):
                    mismatches += 1
                    if mismatches <= 3:
                        print(f"  ❌ Request {i}: Sparse vector keys don't match")
                else:
                    # Compare values for matching keys
                    for key in non_async_sparse.keys():
                        if abs(non_async_sparse[key] - async_sparse[key]) > tolerance:
                            mismatches += 1
                            if mismatches <= 3:
                                print(f"  ❌ Request {i}: Sparse vector value mismatch for key {key}")
                            break
                total_comparisons += 1

            else:
                # For dense and colbert vectors, compare arrays
                non_async_vec = non_async_result.get(vector_type if vector_type != 'dense_vecs' else 'dense_vecs')
                async_vec = async_result.get(vector_type)
                # for i in range(len(non_async_vec)):
                #     print(f"{vector_type}: {i} difference between non-async and async: {non_async_vec[i] - async_vec[i]}")
                if non_async_vec is None or async_vec is None:
                    mismatches += 1
                    if mismatches <= 3:
                        print(f"  ❌ Request {i}: Missing {vector_type}")
                elif not np.allclose(non_async_vec, async_vec, atol=tolerance):
                    mismatches += 1
                    if mismatches <= 3:
                        max_diff = np.max(np.abs(np.array(non_async_vec) - np.array(async_vec)))
                        print(f"  ❌ Request {i}: {vector_type} mismatch (max diff: {max_diff:.2e})")

                total_comparisons += 1

    match_rate = (total_comparisons - mismatches) / total_comparisons * 100
    print(f"  📊 Match rate: {match_rate:.2f}% ({total_comparisons - mismatches}/{total_comparisons} vectors match)")

    if mismatches == 0:
        print(f"  ✅ All embedding vectors match within tolerance!")
        return True
    elif match_rate >= 95.0:
        print(f"  ⚠️  Minor differences found but match rate is good (≥95%)")
        return True
    else:
        print(f"  ❌ Significant mismatches found - match rate too low (<95%)")
        return False


async def compare_async_vs_non_async(model, num_messages=100, max_workers=3):
    """Compare async vs non-async embedding performance"""
    messages = [f"Test message {i}: This is a sample text for embedding comparison" for i in range(num_messages)]

    print(f"\n🎯 === PERFORMANCE COMPARISON ===")
    print(f"Testing {num_messages} embedding requests")

    non_async_results, non_async_time = test_non_async_embedding(model, messages)
    async_results, async_time = await test_async_embedding(model, messages, max_workers=max_workers)
    accuracy_passed = verify_results_match(non_async_results, async_results)

    speedup = non_async_time / async_time
    time_saved = non_async_time - async_time

    print(f"\n📊 === SUMMARY ===")
    print(f"  Non-Async: {non_async_time:.3f}s ({len(messages) / non_async_time:.1f} req/s)")
    print(f"  Async:     {async_time:.3f}s ({len(messages) / async_time:.1f} req/s)")
    print(f"  🚀 Speedup: {speedup:.2f}x faster")
    print(f"  ⏰ Time saved: {time_saved:.3f}s ({time_saved / non_async_time * 100:.1f}% faster)")
    print(f"  ✅ Accuracy: {'PASSED' if accuracy_passed else 'FAILED'}")


if __name__ == "__main__":
    print("🚀 Loading embedding model...")
    embedding_model = BGEM3FlagModel("BAAI/bge-m3", use_fp16=True, devices=["cuda:0", "cuda:1"])
    print("✅ Multi-GPU model loaded!")

    # Warm up model to avoid initialization overhead during timing
    print("🔥 Warming up model...")
    start_time = time.time()
    _ = embedding_model.encode(["Warmup text"] * 5, return_dense=True, return_sparse=True, return_colbert_vecs=True)
    print(f"✅ Model warmed up in {time.time() - start_time:.3f}s")

    # Run the performance comparison
    print("🧪 Running performance comparison...")
    asyncio.run(compare_async_vs_non_async(embedding_model, num_messages=100, max_workers=3))