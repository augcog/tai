from pathlib import Path
import re
from transformers import AutoModelForCausalLM, AutoTokenizer, pipeline
from sentence_transformers import SentenceTransformer
from sklearn.cluster import AgglomerativeClustering
import numpy as np

# 1) Read & split your Markdown into paragraphs
def load_and_split_md(path: Path):
    text = path.read_text(encoding="utf-8")
    # split at two or more newlines
    paras = [p.strip() for p in re.split(r'\n{2,}', text) if p.strip()]
    return paras

# 2) Load a local text-generation model for titles & headers
#    (e.g., a Llama-2 or GPT-2 variant you have locally)
GEN_MODEL = "gpt2-large"              # or "TheBloke/Llama-2-7B-chat" etc.
gen_tok   = AutoTokenizer.from_pretrained(GEN_MODEL)
gen_model = AutoModelForCausalLM.from_pretrained(GEN_MODEL, device_map="auto", torch_dtype="auto")
gen_pipe  = pipeline(
    "text-generation", model=gen_model, tokenizer=gen_tok,
    max_length=64, do_sample=False)

def generate_short_title(paragraph: str) -> str:
    prompt = (
        "Generate a concise title (<=6 words) for this paragraph:\n"
        ">>>" + paragraph.replace("\n", " ") + "\n"
        "Title:"
    )
    out = gen_pipe(prompt, max_new_tokens=16)[0]["generated_text"]
    # grab only the title part
    return out.split("Title:")[-1].strip().split("\n")[0].strip(' "')

# 3) Embed paragraphs and cluster them
embedder = SentenceTransformer("all-MiniLM-L6-v2")
def cluster_paragraphs(paras, n_clusters=None):
    embs = embedder.encode(paras, convert_to_numpy=True, normalize_embeddings=True)
    # if you want to pick n_clusters automatically, you could use
    # a heuristic or set n_clusters=int(len(paras)**0.5) etc.
    n = n_clusters or max(2, int(len(paras)**0.5))
    clust = AgglomerativeClustering(n_clusters=n).fit(embs)
    groups = {}
    for idx, label in enumerate(clust.labels_):
        groups.setdefault(label, []).append(idx)
    return groups

# 4) For each cluster, generate a section header
def generate_section_header(paras: list[str], titles: list[str]) -> str:
    items = "\n".join(f"- {t}: {p[:80]}…" for t,p in zip(titles, paras))
    prompt = (
        "Here are several titled paragraphs:\n"
        f"{items}\n"
        "Generate a concise section header (<=8 words) that captures their common theme:\n"
        "Section header:"
    )
    out = gen_pipe(prompt, max_new_tokens=16)[0]["generated_text"]
    return out.split("Section header:")[-1].strip().split("\n")[0].strip(' "')

# 5) Assemble everything back into Markdown
def build_document(path_in: str, path_out: str):
    paras = load_and_split_md(Path(path_in))
    titles = [generate_short_title(p) for p in paras]

    clusters = cluster_paragraphs(paras)
    md_lines = []
    for label, idxs in clusters.items():
        cluster_paras  = [paras[i] for i in idxs]
        cluster_titles = [titles[i] for i in idxs]
        header = generate_section_header(cluster_paras, cluster_titles)
        md_lines.append(f"# {header}\n")
        for i in idxs:
            md_lines.append(f"## {titles[i]}\n\n{paras[i]}\n")
    Path(path_out).write_text("\n".join(md_lines), encoding="utf-8")
    print(f"Wrote: {path_out}")

if __name__ == "__main__":
    build_document("input.md", "output.md")
