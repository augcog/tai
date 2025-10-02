import logging
import string
import pickle
import uuid
import re
from pathlib import Path
from typing import List, Tuple, Dict, Optional, Iterable, Any

import tiktoken
from file_conversion_router.classes.chunk import Chunk


class Page:
    """
    - Centralized token limits in TOKEN_CFG
    - Unified header detection (MD/HTML) via _is_header_line
    - Unified path-level helpers (_level/_parent) used everywhere
    - Single code-fence span detector reused by both extract & split
    - Safer None handling for file_path/content
    - Removed duplicate regex imports / mini-helpers
    """

    PAGE_LENGTH_THRESHOLD = 20  # (kept if you rely on it elsewhere)

    TOKEN_CFG = {
        "target": 400,
        "min_text": 240,
        "max_text": 500,
        "hard_cap": 480,
        "min_code": 160,
    }

    _MD_ATX = re.compile(r"^\s{0,3}#{1,6}\s+\S")
    _HTML_H = re.compile(r"^\s*<h[1-6]\b", flags=re.I)

    def __init__(
        self,
        course_name: str = "",
        course_code: str = "",
        filetype: str = "",
        page_name: str = "",
        page_url: str = "",
        index_helper: Optional[Dict[Tuple[str, ...], Tuple[int, int]]] = None,
        content: Optional[Dict[str, Any]] = None,
        file_path: Optional[Path] = None,
        file_uuid: Optional[str] = None,
    ):
        self.course_name = course_name
        self.course_code = course_code
        self.page_name = page_name
        self.page_url = page_url
        self.filetype = filetype

        self.index_helper = index_helper or {}
        self.content = content

        self.file_path = Path(file_path)
        self.uuid = file_uuid

        self.chunks: List[Chunk] = []
        self.segments: List[Dict[str, Any]] = []

        self._encoding = None

    # ---------- token utilities ----------
    def _get_encoding(self):
        if self._encoding is None:
            self._encoding = tiktoken.encoding_for_model("gpt-3.5-turbo")
        return self._encoding

    def token_size(self, text: str) -> int:
        enc = self._get_encoding()
        return len(enc.encode(text or ""))

    @staticmethod
    def _level(path_tuple: Tuple[str, ...]) -> int:
        return len(path_tuple) if path_tuple else 0

    @staticmethod
    def _parent(path_tuple: Tuple[str, ...]) -> Tuple[str, ...]:
        return path_tuple[:-1] if path_tuple else tuple()

    @classmethod
    def _same_level_same_parent(cls, a: Tuple[str, ...], b: Tuple[str, ...]) -> bool:
        return cls._level(a) == cls._level(b) and cls._parent(a) == cls._parent(b)

    @classmethod
    def _is_header_line(cls, line: str) -> bool:
        return bool(cls._MD_ATX.match(line or "")) or bool(cls._HTML_H.match(line or ""))

    @staticmethod
    def _rfind_punctuation(s: str, start: int, end: int) -> int:
        for i in range(end - 1, start - 1, -1):
            if s[i] in string.punctuation:
                return i
        return -1

    def recursive_separate(self, text: str, token_limit: int) -> List[str]:
        text = (text or "").strip()
        if not text:
            return []
        n = len(text)
        out: List[str] = []
        start = 0
        while start < n:
            end = start
            while end <= n and self.token_size(text[start:end]) < token_limit:
                end += 1
            if end <= n:
                split_pos = text.rfind("\n\n", start, end)
                if split_pos == -1:
                    split_pos = text.rfind("\n", start, end)
                if split_pos == -1:
                    split_pos = self._rfind_punctuation(text, start, end)
                if split_pos == -1:
                    split_pos = text.rfind(" ", start, end)
                if split_pos == -1 or split_pos <= start:
                    split_pos = end - 1
                out.append(text[start:split_pos].strip())
                start = split_pos + 1
            else:
                out.append(text[start:].strip())
                break
        return [s for s in out if s]

    @staticmethod
    def _compute_code_fence_spans(text: str) -> List[Tuple[int, int]]:
        spans = []
        in_code = False
        start = None
        for i, line in enumerate(text.splitlines(), start=1):
            stripped = line.lstrip()
            if not in_code and stripped.startswith("```"):
                in_code = True
                start = i
            elif in_code and stripped.startswith("```"):
                in_code = False
                spans.append((start, i))
                start = None
        if in_code and start is not None:
            spans.append((start, len(text.splitlines())))
        return spans

    @staticmethod
    def _line_in_spans(line_no: int, spans: Iterable[Tuple[int, int]]) -> bool:
        return any(a <= line_no <= b for a, b in spans)

    def _split_fenced_code(self, fenced_code: str, target_tokens: int, hard_cap: int) -> List[str]:
        lines = fenced_code.splitlines()
        lang = ""
        inner = lines

        if lines and lines[0].lstrip().startswith("```"):
            first = lines[0].strip()
            lang = first[3:].strip(" `")
            end_idx = None
            for i in range(len(lines) - 1, -1, -1):
                if lines[i].lstrip().startswith("```"):
                    end_idx = i
                    break
            inner = lines[1:end_idx] if end_idx is not None and end_idx > 0 else lines[1:]

        n = len(inner)
        pieces: List[str] = []
        start = 0

        while start < n:
            end = start
            last_safe = None
            while end < n:
                candidate = "\n".join(inner[start:end + 1])
                t = self.token_size(candidate)
                if t <= target_tokens:
                    line = inner[end]
                    if line.strip() == "":
                        last_safe = end
                    elif re.match(r"^(def|class)\s+\w+", line):
                        last_safe = end - 1 if end > start else end
                    end += 1
                else:
                    break

            if end == n:
                cut_idx = n - 1
            else:
                cut_idx = last_safe if last_safe is not None and last_safe >= start else max(start, end - 1)

            body = "\n".join(inner[start:cut_idx + 1]).rstrip()
            if self.token_size(body) > hard_cap:
                approx = max(1, (cut_idx + 1 - start) // 2)
                body = "\n".join(inner[start:start + approx]).rstrip()
                cut_idx = start + approx - 1

            pieces.append(f"```{lang}\n{body}\n```")
            start = cut_idx + 1
            while start < n and inner[start].strip() == "":
                start += 1

        return pieces

    def _split_respecting_code_fences(self, body: str, token_limit: int) -> List[Dict[str, str]]:
        lines = (body or "").splitlines()
        segments: List[Dict[str, str]] = []
        text_buf: List[str] = []
        code_buf: List[str] = []
        in_code = False

        def flush_text():
            text = "\n".join(text_buf).strip()
            if text:
                for piece in self.recursive_separate(text, token_limit):
                    segments.append({"content": piece, "kind": "text"})
            text_buf.clear()

        def flush_code():
            code = "\n".join(code_buf).strip()
            if code:
                for piece in self._split_fenced_code(
                    code,
                    target_tokens=token_limit,
                    hard_cap=int(token_limit * 1.2),
                ):
                    segments.append({"content": piece, "kind": "code"})
            code_buf.clear()

        for line in lines:
            stripped = line.strip()
            if not in_code and stripped.startswith("```"):
                in_code = True
                flush_text()
                code_buf.append(line)
            elif in_code:
                code_buf.append(line)
                if stripped.startswith("```"):
                    in_code = False
                    flush_code()
            else:
                text_buf.append(line)

        if in_code:
            flush_code()
        else:
            flush_text()

        return segments

    @staticmethod
    def _first_non_blank_idx(lines: List[str]) -> Optional[int]:
        for i, ln in enumerate(lines):
            if ln.strip():
                return i
        return None

    def _starts_with_header(self, lines: List[str]) -> bool:
        i = self._first_non_blank_idx(lines)
        if i is None:
            return False
        return self._is_header_line(lines[i])

    def _strip_leading_headers(self, lines: List[str]) -> List[str]:
        i, n = 0, len(lines)
        while i < n:
            s = lines[i].strip()
            if not s:
                i += 1
                continue
            if self._is_header_line(lines[i]):
                i += 1
                continue
            break
        return lines[i:]

    def get_sorted_headers_with_valid_line_numbers(self) -> list:
        """
        Get sorted headers from index_helper, filtering out entries where line number is None.
          Returns a list of (path, (page_index, line_number)) tuples sorted by line number.

        Usage:
             all_headers = self.get_sorted_headers_with_valid_line_numbers()
         """
        valid_headers = [(path, value)
        for path, value in self.index_helper.items()if isinstance(value, tuple) and len(value) >= 2 and value[1] is not None]
        return sorted(valid_headers, key=lambda kv: kv[1][1])

    def extract_headers_and_content(self, md_content: str) -> List[Dict[str, Any]]:
        segments: List[Dict[str, Any]] = []
        if not self.index_helper:
            logging.warning("No index helper found, returning empty segments.")
            return segments

        lines = md_content.splitlines()
        total_lines = len(lines)

        # filter out giant code blocks (avoid treating them as section bodies)
        code_spans = self._compute_code_fence_spans(md_content)
        safe_spans = [(a, b) for (a, b) in code_spans if (b - a + 1) / max(1, total_lines) <= 0.7]

        all_headers = self.get_sorted_headers_with_valid_line_numbers()
        sorted_headers = (fh := [
            (p, (i, ln)) for (p, (i, ln)) in all_headers
            if not self._line_in_spans(ln, safe_spans)
        ]) or all_headers

        for i, (path, (page_idx, line_no)) in enumerate(sorted_headers):
            start = max(0, line_no - 1)
            if i + 1 < len(sorted_headers):
                next_line_no = sorted_headers[i + 1][1][1]
                end_exclusive = max(start + 1, next_line_no)
            else:
                end_exclusive = len(lines)

            body_lines = lines[start + 1 : end_exclusive]

            # consecutive headers: skip empty header-only bodies
            if self._starts_with_header(body_lines):
                continue
            body = "\n".join(body_lines).strip()
            if not body:
                continue

            parts = self._split_respecting_code_fences(body, token_limit=self.TOKEN_CFG["target"])
            if not parts:
                # if it's likely HTML/markup blob without fences, keep as text
                if any(ln.lstrip().startswith("<") for ln in body_lines):
                    parts = [{"content": body, "kind": "text"}]
                else:
                    continue

            for part in parts:
                segments.append({
                    "file_path": self.file_path,
                    "page_path": path,
                    "index": page_idx,
                    "content": part["content"],
                    "kind": part["kind"],
                })

        return segments

    def merge_short_segments(
        self,
        segments: List[Dict[str, Any]],
        target_tokens: int,
        min_tokens_text: int,
        max_tokens_text: int,
        hard_cap: int,
        min_tokens_code: int,
    ) -> List[Dict[str, Any]]:

        used = [False] * len(segments)
        out: List[Dict[str, Any]] = []
        i = 0

        def same_family(cur_idx: int, k: int) -> bool:
            if k < 0 or k >= len(segments) or used[k]:
                return False
            cur, s = segments[cur_idx], segments[k]
            return (
                self._level(s["page_path"]) == self._level(cur["page_path"])
                and self._parent(s["page_path"]) == self._parent(cur["page_path"])
            )

        while i < len(segments):
            if used[i]:
                i += 1
                continue

            cur = segments[i]
            cur_tokens = self.token_size(cur["content"])
            need_pack = cur_tokens < (min_tokens_code if cur["kind"] == "code" else min_tokens_text)

            if not need_pack:
                out.append(cur)
                used[i] = True
                i += 1
                continue

            group = [i]
            total = cur_tokens
            L, R = i - 1, i + 1

            def candidate_list():
                cands = []
                for k in (L, R):
                    if not same_family(i, k):
                        continue
                    s = segments[k]
                    new_total = total + self.token_size(s["content"]) + 2
                    if new_total > hard_cap:
                        continue
                    score = abs(new_total - target_tokens)
                    # prefer text when mixing, preserve code granularity
                    weight = 0 if s["kind"] == "text" else 1
                    cands.append((score, weight, k, new_total))
                cands.sort(key=lambda x: (x[0], x[1]))
                return cands

            while total < min_tokens_text:
                cands = candidate_list()
                if not cands:
                    break
                _, _, pick, new_total = cands[0]
                # don't overshoot too much once we're decent length
                if new_total > max_tokens_text and total >= int(0.6 * target_tokens):
                    break
                if pick == L:
                    group.insert(0, L)
                    L -= 1
                else:
                    group.append(R)
                    R += 1
                total = new_total

            contents = []
            min_index = cur["index"]
            kinds = set()
            for k in group:
                used[k] = True
                contents.append(segments[k]["content"])
                min_index = min(min_index, segments[k]["index"])
                kinds.add(segments[k]["kind"])

            merged_kind = "text" if kinds == {"text"} else ("code" if kinds == {"code"} else "mixed")

            merged = {
                "file_path": cur["file_path"],
                "page_path": cur["page_path"],
                "index": min_index,
                "content": "\n\n".join(contents),
                "kind": merged_kind,
            }
            out.append(merged)
            i += 1

        # Final safety split for overly-long text chunks
        final: List[Dict[str, Any]] = []
        for seg in out:
            if seg["kind"] == "text" and self.token_size(seg["content"]) > max_tokens_text:
                for piece in self.recursive_separate(seg["content"], max_tokens_text):
                    final.append({
                        "file_path": seg["file_path"],
                        "page_path": seg["page_path"],
                        "index": seg["index"],
                        "content": piece,
                        "kind": "text",
                    })
            else:
                final.append(seg)

        return final

    def page_separate_to_segments(self) -> None:
        base = self.extract_headers_and_content(self.content.get("text", ""))
        cfg = self.TOKEN_CFG
        self.segments = self.merge_short_segments(
            base,
            target_tokens=cfg["target"],
            min_tokens_text=cfg["min_text"],
            max_tokens_text=cfg["max_text"],
            hard_cap=cfg["hard_cap"],
            min_tokens_code=cfg["min_code"],
        )

    def segments_to_chunks(self, start_index: int = 1) -> List[Chunk]:
        self.chunks = []
        clean_segments: List[Dict[str, Any]] = []

        for seg in self.segments:
            lines = (seg["content"] or "").splitlines()
            lines = self._strip_leading_headers(lines)
            s = dict(seg)
            s["content"] = "\n".join(lines).strip()
            clean_segments.append(s)

        for cidx, seg in enumerate(clean_segments, start=start_index):
            header = seg["page_path"]
            index = seg["index"]
            content = seg["content"]
            file_path: Optional[Path] = seg.get("file_path") or self.file_path

            # robust name and reference path
            name = file_path.name if isinstance(file_path, Path) else ""
            parts = [name, header[0], header[-1]] if len(header) > 2 else [name, *header]
            reference_path = " > ".join([p for p in parts if p])

            self.chunks.append(
                Chunk(
                    chunk_index=cidx,
                    content=content,
                    titles=header,
                    chunk_url=self.page_url,
                    index=index,
                    is_split=False,
                    file_path=file_path,
                    file_uuid=self.uuid,
                    chunk_uuid=self.gen_chunk_uuid(),
                    reference_path=reference_path,
                    course_name=self.course_name,
                    course_code=self.course_code,
                )
            )
        return self.chunks

    def gen_chunk_uuid(self) -> str:
        return str(uuid.uuid4())

    def to_chunk(self) -> List[Chunk]:
        self.page_separate_to_segments()
        return self.segments_to_chunks()

    def chunks_to_pkl(self, output_path: str) -> None:
        with open(output_path, "wb") as f:
            pickle.dump(self.chunks, f)
