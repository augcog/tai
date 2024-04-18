import os
import pickle
import openai
from VideoProcess import process_video_scenes
from TranscriptProcess import transcribe_audio_with_whisper
import tiktoken

class Segment:
    def __init__(self, base_path, video_urls):
        self.base_path = base_path
        self.video_urls = video_urls
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
        for seg_start, seg_end in seg_time:
            paragraph = [chunk['text'] for chunk in transcript if seg_start <= chunk['start'] <= seg_end]
            paragraphs.append((paragraph, seg_start))
        return paragraphs

    def save_content_to_pkl(self, dict_list, filename):
        with open(filename, 'wb') as file:
            pickle.dump(dict_list, file)

    def process_video_audio(self):
        for (root, dirs, files), video_url in zip(os.walk(self.base_path), self.video_urls):
            if not files:
                continue
            dict_list = []
            folder = os.path.basename(root)
            video, audio = None, None
            for file in files:
                if file.endswith('.mp4'):
                    video = file
                elif file.endswith('.wav'):
                    audio = file

            if video and audio:
                video_path = os.path.join(root, video)
                audio_path = os.path.join(root, audio)
                seg_time = process_video_scenes(video_path)
                transcript = transcribe_audio_with_whisper(audio_path)
                paragraphs = self.paragraph_generator(transcript, seg_time)
                for i, time in paragraphs:
                    paragraph = ''.join(i)
                    if paragraph:
                        title = self.title_with_chat_completion("zephyr", paragraph)
                        print(time)
                        print(title)

                        dict_list.append({'Page_table': '', 'Page_path': title, 'Segment_print': paragraph, 'url': video_url, 'time': int(time)})
                save_path = os.path.join(root, f'{folder}_content.pkl')
                self.save_content_to_pkl(dict_list, save_path)
            else:
                print(f"No video or audio files found in {root}. Skipping.")

# Example of using the class
base_path = "/path/to/videos"
video_urls = ["http://example.com/video1", "http://example.com/video2"]
segmenter = Segment(base_path, video_urls)
segmenter.process_video_audio()
