import logging
import string
import tiktoken
import pickle
from pathlib import Path
import uuid
from file_conversion_router.classes.chunk import Chunk

class Page:
    PAGE_LENGTH_THRESHOLD = 20

    def __init__(self,
                 course_name: str = "",
                 course_id: str = "",
                 filetype: str = "",
                 page_name: str = "",
                 page_url: str = "",
                 index_helper: dict = None,
                 content: dict = None,
                 file_path=None,
                 file_uuid: str = None):
        self.course_name = course_name
        self.course_id = course_id
        self.page_name = page_name
        self.content = content
        self.chunks = []
        self.page_url = page_url
        self.filetype = filetype
        self.index_helper = index_helper
        self.segments = []
        self.file_path = Path(file_path)
        self.uuid = file_uuid

    def token_size(self, sentence: str) -> int:
        encoding = tiktoken.encoding_for_model("gpt-3.5-turbo")
        return len(encoding.encode(sentence or ""))

    def _header_level(self, path_tuple) -> int:
        return len(path_tuple) if path_tuple else 0

    def _parent_path(self, path_tuple):
        return path_tuple[:-1] if path_tuple else tuple()

    def _same_level_same_parent(self, a, b) -> bool:
        return (self._header_level(a) == self._header_level(b)) and (self._parent_path(a) == self._parent_path(b))

    def recursive_separate(self, response: str, token_limit: int = 400) -> list:

        def rfind_punctuation(s: str, start: int, end: int) -> int:
            for i in range(end - 1, start - 1, -1):
                if s[i] in string.punctuation:
                    return i
            return -1

        response = (response or "").strip()
        if not response:
            return []
        n = len(response)
        out = []
        start = 0
        while start < n:
            end = start
            while end <= n and self.token_size(response[start:end]) < token_limit:
                end += 1
            if end <= n:
                split_pos = response.rfind("\n\n", start, end)
                if split_pos == -1:
                    split_pos = response.rfind("\n", start, end)
                if split_pos == -1:
                    split_pos = rfind_punctuation(response, start, end)
                if split_pos == -1:
                    split_pos = response.rfind(" ", start, end)
                if split_pos == -1 or split_pos <= start:
                    split_pos = end - 1
                out.append(response[start:split_pos].strip())
                start = split_pos + 1
            else:
                out.append(response[start:].strip())
                break
        return [s for s in out if s]

    def _split_respecting_code_fences(self, body: str, token_limit: int = 400):
        lines = (body or "").splitlines()
        segments = []
        text_buf, code_buf = [], []
        in_code = False

        def flush_text_buf():
            text = "\n".join(text_buf).strip()
            if text:
                for piece in self.recursive_separate(text, token_limit):
                    segments.append({"content": piece, "kind": "text"})
            text_buf.clear()

        def flush_code_buf():
            code = "\n".join(code_buf).strip()
            if code:
                for piece in self._split_fenced_code(code, target_tokens=token_limit,
                                                     hard_cap=int(token_limit * 1.2)):
                    segments.append({"content": piece, "kind": "code"})
            code_buf.clear()

        for line in lines:
            stripped = line.strip()
            if not in_code and stripped.startswith("```"):
                in_code = True
                flush_text_buf()
                code_buf.append(line)
            elif in_code:
                code_buf.append(line)
                if stripped.startswith("```"):
                    in_code = False
                    flush_code_buf()
            else:
                text_buf.append(line)

        if in_code:
            flush_code_buf()
        else:
            flush_text_buf()

        return segments

    def extract_headers_and_content(self, md_content: str):
        segments = []
        lines = md_content.splitlines()

        if not self.index_helper:
            logging.warning("No index helper found, returning empty segments.")
            return segments

        code_spans = self._compute_code_fence_spans(md_content)

        sorted_headers = sorted(self.index_helper.items(), key=lambda item: item[1][1])  # 按 line_no
        sorted_headers = [
            (path, (page_idx, line_no))
            for (path, (page_idx, line_no)) in sorted_headers
            if not self._line_in_spans(line_no, code_spans)
        ]

        if not sorted_headers:
            return segments

        for i, (path, (page_idx, line_no)) in enumerate(sorted_headers):
            start = line_no - 1  # 0-based
            if i + 1 < len(sorted_headers):
                next_line_no = sorted_headers[i + 1][1][1]
                end = max(start + 1, next_line_no - 1)
            else:
                end = len(lines)

            body_lines = lines[start + 1:end]
            body = "\n".join(body_lines).strip()
            if not body:
                continue

            parts = self._split_respecting_code_fences(body, token_limit=400)
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
            segments: list,
            target_tokens: int = 400,
            min_tokens: int = 240,
            max_tokens: int = 430,
            hard_cap: int = 480,
            min_tokens_code: int = 160,
    ):

        def lvl(p):
            return len(p) if p else 0

        def parent(p):
            return p[:-1] if p else tuple()

        used = [False] * len(segments)
        out = []
        i = 0

        while i < len(segments):
            if used[i]:
                i += 1
                continue

            cur = segments[i]
            cur_tokens = self.token_size(cur["content"])
            cur_parent = parent(cur["page_path"])
            cur_level = lvl(cur["page_path"])

            def same_family(k):
                if k < 0 or k >= len(segments) or used[k]:
                    return False
                s = segments[k]
                return (lvl(s["page_path"]) == cur_level) and (parent(s["page_path"]) == cur_parent)

            need_pack = False
            if cur["kind"] == "code":
                need_pack = (cur_tokens < min_tokens_code)
            else:
                need_pack = (cur_tokens < min_tokens)

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
                    if not same_family(k):
                        continue
                    s = segments[k]
                    new_total = total + self.token_size(s["content"]) + 2
                    if new_total > hard_cap:
                        continue
                    score = abs(new_total - target_tokens)
                    weight = 0 if s["kind"] == "text" else 1
                    cands.append((score, weight, k, new_total))
                cands.sort(key=lambda x: (x[0], x[1]))
                return cands

            while total < min_tokens:
                cands = candidate_list()
                if not cands:
                    break
                _, _, pick, new_total = cands[0]
                if new_total > max_tokens and total >= int(0.6 * target_tokens):
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
            for k in group:
                used[k] = True
                contents.append(segments[k]["content"])
                min_index = min(min_index, segments[k]["index"])

            kinds = {segments[k]["kind"] for k in group}
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

        final = []
        for seg in out:
            if seg["kind"] == "text" and self.token_size(seg["content"]) > max_tokens:
                for piece in self.recursive_separate(seg["content"], max_tokens):
                    final.append({
                        "file_path": seg["file_path"],
                        "page_path": seg["page_path"],
                        "index": seg["index"],
                        "content": piece,
                        "kind": "text"
                    })
            else:
                final.append(seg)

        return final

    def page_separate_to_segments(self) -> None:
        base_segments = self.extract_headers_and_content(self.content["text"])
        self.segments = self.merge_short_segments(
            base_segments,
            target_tokens=400,
            min_tokens=240,
            max_tokens=430,
            hard_cap=480,
            min_tokens_code=160,
        )

    def segments_to_chunks(self, start_index: int = 1):
        self.chunks = []

        for cidx, seg in enumerate(self.segments, start=start_index):
            header = seg["page_path"]
            index = seg["index"]
            content = seg["content"]
            file_path = seg.get("file_path")

            name = file_path.name if file_path else ""
            if len(header) > 2:
                parts = [name, header[0], header[-1]]
            else:
                parts = [name, *header]
            reference_path = " > ".join(parts)

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
                    course_id=self.course_id,
                )
            )
        return self.chunks

    def gen_chunk_uuid(self) -> str:
        return str(uuid.uuid4())

    def to_chunk(self) -> list:
        self.page_separate_to_segments()
        self.chunks = self.segments_to_chunks()
        return self.chunks

    def chunks_to_pkl(self, output_path: str) -> None:
        with open(output_path, "wb") as f:
            pickle.dump(self.chunks, f)

    def _compute_code_fence_spans(self, text: str):
        spans = []
        in_code = False
        start = None
        lines = text.splitlines()
        for i, line in enumerate(lines, start=1):
            stripped = line.lstrip()
            if not in_code and stripped.startswith("```"):
                in_code = True
                start = i
            elif in_code and stripped.startswith("```"):
                in_code = False
                spans.append((start, i))
                start = None
        if in_code and start is not None:
            spans.append((start, len(lines)))
        return spans

    def _line_in_spans(self, line_no: int, spans) -> bool:
        for a, b in spans:
            if a <= line_no <= b:
                return True
        return False

    def _split_fenced_code(self, fenced_code: str,
                           target_tokens: int = 400,
                           hard_cap: int = 480) -> list[str]:

        import re

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
        pieces = []
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
                if last_safe is not None and last_safe >= start:
                    cut_idx = last_safe
                else:
                    cut_idx = max(start, end - 1)

            body = "\n".join(inner[start:cut_idx + 1]).rstrip()

            if self.token_size(body) > hard_cap:
                approx_lines = max(1, (cut_idx + 1 - start) // 2)
                body = "\n".join(inner[start:start + approx_lines]).rstrip()
                cut_idx = start + approx_lines - 1

            pieces.append(f"```{lang}\n{body}\n```")
            start = cut_idx + 1
            while start < n and inner[start].strip() == "":
                start += 1

        return pieces
