from wtpsplit import WtP

wtp = WtP("wtp-bert-mini")
# optionally run on GPU for better performance
# also supports TPUs via e.g. wtp.to("xla:0"), in that case pass `pad_last_batch=True` to wtp.split
wtp.half().to("cuda")
with open("pure_text.txt", "r") as f:
    text = f.read()
passage = wtp.split(text, do_paragraph_segmentation=True)
for id in range(len(passage)):
    paragraph = ""
    for sentence in passage[id]:
        paragraph += f"{sentence}\n"
    print(f"Paragraph {id}: {paragraph}")