import json
import collections
import yaml

# 1. 读取 Markdown 文件内容（假设文件名为 file.md）
with open('file.md', 'r', encoding='utf-8') as f_md:
    md_content = f_md.read()

# 2. 读取 JSON 文件内容（假设文件名为 file.json）
with open('file.json', 'r', encoding='utf-8') as f_json:
    json_list = json.load(f_json)

# 3. 将 JSON 对象按 page_idx 分组
pages = collections.defaultdict(list)
for item in json_list:
    page_idx = item.get("page_idx")
    pages[page_idx].append(item)

# 用于存储每个页面的起始行号，结构为 {page_idx: start_line}
pages_start_lines = {}

# 4. 遍历每个页面，寻找该页面中各文本段在 Markdown 文件中的最早位置
for page_idx, items in pages.items():
    page_start_line = None  # 用于保存该页面中最小的行号
    for item in items:
        text_snippet = item.get("text", "").strip()
        # 如果文本为空，则跳过
        if not text_snippet:
            continue

        # 在 Markdown 文件中查找该文本段的位置
        pos = md_content.find(text_snippet)
        if pos == -1:
            # 没有找到匹配的文本，则跳过（或记录日志）
            continue

        # 计算该文本段在 Markdown 文件中的起始行号
        line_num = md_content[:pos].count('\n') + 1

        # 更新页面的起始行（取最小的行号）
        if page_start_line is None or line_num < page_start_line:
            page_start_line = line_num

    # 如果该页面有匹配到文本，记录起始行；否则可置为 None 或其他默认值
    pages_start_lines[page_idx] = page_start_line

# 5. 将 pages_start_lines 写入一个 YAML 文件（例如 pages.yaml）
with open('pages.yaml', 'w', encoding='utf-8') as f_yaml:
    # 使用 safe_dump，并允许 Unicode 字符（如中文）写入
    yaml.safe_dump(pages_start_lines, f_yaml, allow_unicode=True)

print("YAML 文件写入成功，内容如下：")
print(yaml.safe_dump(pages_start_lines, allow_unicode=True))


