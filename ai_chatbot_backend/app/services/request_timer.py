"""Request timing utility for tracking latency across the question-answering flow."""
import time
from typing import Dict, Optional
from dataclasses import dataclass, field


@dataclass
class RequestTimer:
    """Track timing milestones for a single request.

    Usage:
        timer = RequestTimer(request_id="abc123")
        timer.mark("embedding_start")
        # ... do embedding ...
        timer.mark("embedding_end")
        print(timer.report())
    """
    request_id: str
    events: Dict[str, float] = field(default_factory=dict)
    start_time: float = field(default_factory=time.time)

    def mark(self, event_name: str) -> float:
        """Record a timing event, return elapsed time since start."""
        elapsed = time.time() - self.start_time
        self.events[event_name] = elapsed
        return elapsed

    def get_interval(self, start_event: str, end_event: str) -> Optional[float]:
        """Get time interval between two events."""
        if start_event in self.events and end_event in self.events:
            return self.events[end_event] - self.events[start_event]
        return None

    def report(self) -> str:
        """Generate timing report with all events and key metrics."""
        lines = [f"\n[TIMING] Request {self.request_id}:"]

        # Print all events in chronological order
        for event, elapsed in sorted(self.events.items(), key=lambda x: x[1]):
            lines.append(f"  {event}: {elapsed:.3f}s")

        # Calculate and print key metrics
        lines.append("\n[TIMING] Key Metrics:")

        embedding_time = self.get_interval("embedding_start", "embedding_end")
        if embedding_time is not None:
            lines.append(f"  Embedding time: {embedding_time:.3f}s")

        retrieval_time = self.get_interval("retrieval_start", "retrieval_end")
        if retrieval_time is not None:
            lines.append(f"  Retrieval time: {retrieval_time:.3f}s")

        if "first_token" in self.events:
            lines.append(f"  Time to First Token (TTFT): {self.events['first_token']:.3f}s")

        if "llm_generation_start" in self.events:
            lines.append(f"  Preprocessing total: {self.events['llm_generation_start']:.3f}s")

        return "\n".join(lines)
