import os
import pickle
<<<<<<< HEAD
import openai
from VideoProcess import process_video_scenes
from TranscriptProcess import transcribe_audio_with_whisper
import tiktoken

=======
from transformers import pipeline, set_seed, AutoTokenizer, AutoModelForCausalLM
import torch
from VideoProcess import process_video_scenes
from TranscriptProcess import transcribe_audio_with_whisper
import tiktoken
import dotenv

dotenv.load_dotenv()
>>>>>>> cdbc2f5496cabb88b8715e1213624541579ec1fc
class Segment:
    def __init__(self, base_path, video_urls):
        self.base_path = base_path
        self.video_urls = video_urls
<<<<<<< HEAD
        self.openai_api_key = os.getenv("OPENAI_API_KEY")

    def title_with_chat_completion(self, model, text):
        if model == 'local' or model == 'zephyr':
            openai.api_key = "empty"
            openai.api_base = "http://localhost:8000/v1"
        elif model == 'openai':
            openai.api_key = self.openai_api_key
        system_prompt = "Generate a title for the following text. Only give a title with no explanation. Only give a title with no explanatioin."
        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": f"text:{text}"}
        ]
        response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=messages,
        )
        return response["choices"][0]["message"]["content"]
=======
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
>>>>>>> cdbc2f5496cabb88b8715e1213624541579ec1fc

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
<<<<<<< HEAD
=======
        if not seg_time:
            return [([chunk['text'] for chunk in transcript], 0)]
>>>>>>> cdbc2f5496cabb88b8715e1213624541579ec1fc
        for seg_start, seg_end in seg_time:
            paragraph = [chunk['text'] for chunk in transcript if seg_start <= chunk['start'] <= seg_end]
            paragraphs.append((paragraph, seg_start))
        return paragraphs

    def save_content_to_pkl(self, dict_list, filename):
        with open(filename, 'wb') as file:
            pickle.dump(dict_list, file)
<<<<<<< HEAD

    def process_video_audio(self):
        for (root, dirs, files), video_url in zip(os.walk(self.base_path), self.video_urls):
            if not files:
                continue
            dict_list = []
            folder = os.path.basename(root)
            video, audio = None, None
            for file in files:
=======
    def get_directory(self):
        return sorted([d for d in os.listdir(self.base_path) if os.path.isdir(os.path.join(self.base_path, d))])
    def process_video_audio(self):
        print("list of: ")
        print(self.get_directory())
        for folder, video_url in zip(self.get_directory(), self.video_urls):
            dict_list = []
            video, audio = None, None
            for file in os.listdir(os.path.join(self.base_path,folder)):
>>>>>>> cdbc2f5496cabb88b8715e1213624541579ec1fc
                if file.endswith('.mp4'):
                    video = file
                elif file.endswith('.wav'):
                    audio = file

            if video and audio:
<<<<<<< HEAD
                video_path = os.path.join(root, video)
                audio_path = os.path.join(root, audio)
=======
                video_path = os.path.join(self.base_path,folder, video)
                audio_path = os.path.join(self.base_path,folder, audio)
>>>>>>> cdbc2f5496cabb88b8715e1213624541579ec1fc
                seg_time = process_video_scenes(video_path)
                transcript = transcribe_audio_with_whisper(audio_path)
                paragraphs = self.paragraph_generator(transcript, seg_time)
                for i, time in paragraphs:
                    paragraph = ''.join(i)
                    if paragraph:
<<<<<<< HEAD
                        title = self.title_with_chat_completion("zephyr", paragraph)
=======
                        title = self.title_with_chat_completion(paragraph)
>>>>>>> cdbc2f5496cabb88b8715e1213624541579ec1fc
                        print(time)
                        print(title)

                        dict_list.append({'Page_table': '', 'Page_path': title, 'Segment_print': paragraph, 'url': video_url, 'time': int(time)})
<<<<<<< HEAD
                save_path = os.path.join(root, f'{folder}_content.pkl')
                self.save_content_to_pkl(dict_list, save_path)
            else:
                print(f"No video or audio files found in {root}. Skipping.")

# Example of using the class
base_path = "/path/to/videos"
video_urls = ["http://example.com/video1", "http://example.com/video2"]
segmenter = Segment(base_path, video_urls)
segmenter.process_video_audio()
=======
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
>>>>>>> cdbc2f5496cabb88b8715e1213624541579ec1fc
