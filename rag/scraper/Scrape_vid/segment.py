import os
import pickle
from transformers import pipeline, set_seed, AutoTokenizer, AutoModelForCausalLM
import torch
from VideoProcess import process_video_scenes
from TranscriptProcess import transcribe_audio_with_whisper
import tiktoken
import dotenv

dotenv.load_dotenv()
class Segment:
    def __init__(self, base_path, video_urls):
        self.base_path = base_path
        self.video_urls = video_urls
        self.model_id = "meta-llama/Meta-Llama-3-8B-Instruct"
        # Initialize the tokenizer and model only once when the classes is instantiated
        self.auto_tokenizer = AutoTokenizer.from_pretrained(self.model_id)
        self.pipeline = pipeline(
            'text-generation',
            model=self.model_id,
            model_kwargs={"torch_dtype": torch.bfloat16},
            device="cuda"
        )  # Assumes a GPU is available at device 0

    def title_with_chat_completion(self, text):
        terminators = [
            self.pipeline.tokenizer.eos_token_id,
            self.pipeline.tokenizer.convert_tokens_to_ids("<|eot_id|>")
        ]

        messages = [
            {"role": "system", "content": "Generate a title for the following text. Only answer one title"},
            {"role": "user", "content": text}
        ]
        prompt = self.pipeline.tokenizer.apply_chat_template(
            messages,
            tokenize=False,
            add_generation_prompt=True
        )
        outputs = self.pipeline(
            prompt,
            max_new_tokens=1000,  # Adjust based on desired title length
            eos_token_id=terminators,
            do_sample=True
        )
        title = outputs[0]["generated_text"][len(prompt):].replace('\n', ' ').strip()
        return title

    # Remainder of your methods...

    def token_size(self, sentence):
        encoding = tiktoken.encoding_for_model("gpt-3.5-turbo")
        return len(encoding.encode(sentence))

    def process_paragraphs(self, paragraphs):
        for i in range(len(paragraphs) - 1):
            if not paragraphs[i].endswith('.'):
                sentences = paragraphs[i + 1].split('.')
                paragraphs[i] += ' ' + sentences[0] + '.'
                if len(sentences) > 1:
                    paragraphs[i + 1] = '.'.join(sentences[1:])
                    if paragraphs[i + 1]:
                        paragraphs[i + 1] = '.' + paragraphs[i + 1]
                else:
                    paragraphs[i + 1] = ''
        return paragraphs

    def paragraph_generator(self, transcript, seg_time):
        paragraphs = []
        if not seg_time:
            return [([chunk['text'] for chunk in transcript], 0)]
        for seg_start, seg_end in seg_time:
            paragraph = [chunk['text'] for chunk in transcript if seg_start <= chunk['start'] <= seg_end]
            paragraphs.append((paragraph, seg_start))
        return paragraphs

    def save_content_to_pkl(self, dict_list, filename):
        with open(filename, 'wb') as file:
            pickle.dump(dict_list, file)
    def get_directory(self):
        return sorted([d for d in os.listdir(self.base_path) if os.path.isdir(os.path.join(self.base_path, d))])
    def process_video_audio(self):
        print("list of: ")
        print(self.get_directory())
        for folder, video_url in zip(self.get_directory(), self.video_urls):
            dict_list = []
            video, audio = None, None
            for file in os.listdir(os.path.join(self.base_path,folder)):
                if file.endswith('.mp4'):
                    video = file
                elif file.endswith('.wav'):
                    audio = file

            if video and audio:
                video_path = os.path.join(self.base_path,folder, video)
                audio_path = os.path.join(self.base_path,folder, audio)
                seg_time = process_video_scenes(video_path)
                transcript = transcribe_audio_with_whisper(audio_path)
                paragraphs = self.paragraph_generator(transcript, seg_time)
                for i, time in paragraphs:
                    paragraph = ''.join(i)
                    if paragraph:
                        title = self.title_with_chat_completion(paragraph)
                        print(time)
                        print(title)

                        dict_list.append({'Page_table': '', 'Page_path': title, 'Segment_print': paragraph, 'url': video_url, 'time': int(time)})
                save_path = os.path.join(self.base_path,folder, f'{folder}_content.pkl')
                self.save_content_to_pkl(dict_list, save_path)
            else:
                print(f"No video or audio files found in {self.base_path}. Skipping.")
        
    def process_video_audio_to_md(self):
        print("list of: ")
        print(self.get_directory())
        for folder, video_url in zip(self.get_directory(), self.video_urls):
            dict_list = []
            video, audio = None, None
            for file in os.listdir(os.path.join(self.base_path, folder)):
                if file.endswith('.mp4'):
                    video = file
                elif file.endswith('.wav'):
                    audio = file

            if video and audio:
                video_path = os.path.join(self.base_path, folder, video)
                audio_path = os.path.join(self.base_path, folder, audio)
                seg_time = process_video_scenes(video_path)
                transcript = transcribe_audio_with_whisper(audio_path)
                paragraphs = self.paragraph_generator(transcript, seg_time)
                md_path = os.path.join(self.base_path, folder, f'{folder}_content.md')
                print(f"Now handling {folder}")
                with open(md_path, 'w') as md_file:
                    for i, time in paragraphs:
                        paragraph = ''.join(i)
                        if paragraph:
                            title = self.title_with_chat_completion(paragraph)
                            print(time)
                            print(title)

                            # Write the title and paragraph to the Markdown file
                            md_file.write(f'# {title}\n\n')
                            md_file.write(f'{paragraph}\n\n')

            else:
                print(f"No video or audio files found in {self.base_path}. Skipping.")

# Example of using the classes
# base_path = "/path/to/videos"
# video_urls = ["http://example.com/video1", "http://example.com/video2"]
# segmenter = Segment(base_path, video_urls)
# segmenter.process_video_audio()
