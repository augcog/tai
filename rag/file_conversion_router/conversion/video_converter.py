from pathlib import Path
import time
import whisper
from rag.file_conversion_router.conversion.base_converter import BaseConverter
from rag.file_conversion_router.classes.vidpage import VidPage
from rag.file_conversion_router.classes.page import Page
from moviepy.editor import AudioFileClip
from scenedetect import open_video, SceneManager
from scenedetect.detectors import AdaptiveDetector
from scenedetect.scene_manager import save_images, write_scene_list
from transformers import pipeline, set_seed, AutoTokenizer, AutoModelForCausalLM
import torch
import os
import yaml

class VideoConverter(BaseConverter):
    model_id = "meta-llama/Meta-Llama-3-8B-Instruct"
    auto_tokenizer = None
    pipeline = None
    def __init__(self):
        super().__init__()
        # Initialize the tokenizer and model only once when the classes is instantiated
        self.paragraphs = []

    @staticmethod
    def initialize_static_resources():
        if VideoConverter.auto_tokenizer is None:
            VideoConverter.auto_tokenizer = AutoTokenizer.from_pretrained(VideoConverter.model_id)

            VideoConverter.pipeline = pipeline(
                'text-generation',
                model=VideoConverter.model_id,
                model_kwargs={"torch_dtype": torch.bfloat16},
                device="cuda"
            )
    def title_with_chat_completion(self, text):
        VideoConverter.initialize_static_resources()
        terminators = [
            VideoConverter.pipeline.tokenizer.eos_token_id,
            VideoConverter.pipeline.tokenizer.convert_tokens_to_ids("<|eot_id|>")
        ]

        messages = [
            {"role": "system", "content": "Generate a title for the following text. Only answer one title"},
            {"role": "user", "content": text}
        ]
        prompt = VideoConverter.pipeline.tokenizer.apply_chat_template(
            messages,
            tokenize=False,
            add_generation_prompt=True
        )
        outputs = VideoConverter.pipeline(
            prompt,
            max_new_tokens=1000,  # Adjust based on desired title length
            eos_token_id=terminators,
            do_sample=True
        )
        title = outputs[0]["generated_text"][len(prompt):].replace('\n', ' ').strip()
        return title
    def transcribe_audio_with_whisper(self, audio_file_path):
        print("Loading Whisper model...")
        start_time = time.time()
        model = whisper.load_model("base")

        print(f"Transcribing {audio_file_path}...")
        result = model.transcribe(audio_file_path)

        segments = []
        if "segments" in result:
            for segment in result["segments"]:
                segment_dict = {
                    "start": segment["start"],
                    "end": segment["end"],
                    "text": segment["text"]
                }
                segments.append(segment_dict)

        print(f"Transcription completed in {time.time() - start_time} seconds.")
        return segments

    def convert_mp4_to_wav(self, mp4_file_path, output_path):
        wav_file_path = mp4_file_path.with_suffix(".wav")
        print(mp4_file_path)
        audio_clip = AudioFileClip(str(mp4_file_path))  # Load the audio track from the MP4 file
        audio_clip.write_audiofile(str(wav_file_path))  # Save the audio as a WAV file
        audio_clip.close()  # Close the clip to free resources
        return wav_file_path

    def process_video_scenes(self, video_path, output_path):
        def setup_scene_detection(video_path, custom_window_width=50, custom_weights=None):
            video = open_video(video_path)
            scene_manager = SceneManager()

            video_folder = os.path.dirname(video_path)
            images_folder_name = os.path.splitext(os.path.basename(video_path))[0] + "_images"
            images_output_dir = os.path.join(video_folder, images_folder_name)

            os.makedirs(images_output_dir, exist_ok=True)

            if custom_weights is None:
                custom_weights = AdaptiveDetector.Components(delta_hue=0.1, delta_sat=1.0, delta_lum=1.0,
                                                             delta_edges=1.0)

            adaptive_detector = AdaptiveDetector(
                window_width=custom_window_width,
                weights=custom_weights,
            )

            scene_manager.add_detector(adaptive_detector)

            return video, scene_manager, images_output_dir

        def detect_scenes_and_save_images(video, scene_manager, images_output_dir):
            scene_manager.detect_scenes(video, show_progress=True)

            scene_list = scene_manager.get_scene_list()
            csv_file_path = os.path.join(images_output_dir, "detected_scenes.csv")
            with open(csv_file_path, 'w', newline='') as csv_file:
                write_scene_list(csv_file, scene_list)

            print(f"Scene list saved to {csv_file_path}")

            image_filenames = save_images(
                scene_list=scene_list,
                video=video,
                num_images=3,
                output_dir=images_output_dir,
            )

            # Prepare the list of start and end times
            scene_times = [(start_time.get_seconds(), end_time.get_seconds()) for start_time, end_time in scene_list]

            for i, ((start_time, end_time), images) in enumerate(zip(scene_list, image_filenames.values()), start=1):
                print(f"Scene {i}: Start Time: {start_time.get_seconds()}, End Time: {end_time.get_seconds()}")
                for image_path in images:
                    print(f"  - {image_path}")
            return scene_times

        video, scene_manager, images_output_dir = setup_scene_detection(str(video_path))
        scene_times = detect_scenes_and_save_images(video, scene_manager, images_output_dir)
        return scene_times

    def paragraph_generator(self, transcript, seg_time):
        paragraphs = []
        if not seg_time:
            return [([chunk['text'] for chunk in transcript], 0)]
        for seg_start, seg_end in seg_time:
            paragraph = [chunk['text'] for chunk in transcript if seg_start <= chunk['start'] <= seg_end]
            paragraphs.append((paragraph, seg_start))
        self.paragraphs = paragraphs
        return paragraphs

    def _to_markdown(self, input_path, output_path):
        audio = self.convert_mp4_to_wav(input_path, output_path)
        seg_time = self.process_video_scenes(input_path, output_path)
        transcript = self.transcribe_audio_with_whisper(str(audio))
        paragraphs = self.paragraph_generator(transcript, seg_time)
        markdown_content = ""
        for i, (paragraph, time) in enumerate(paragraphs):
            paragraph_text = ''.join(paragraph)
            if paragraph_text:
                title = self.title_with_chat_completion(paragraph_text)
                markdown_content += f'# {title}\n\n'
                markdown_content += f'{paragraph_text}\n\n'
        md_path = output_path.with_suffix(".md")
        with open(md_path, 'w') as md_file:
            md_file.write(markdown_content)
        return md_path

    def _to_page(self, input_path: Path, output_path: Path) -> Page:
        """Perform mp4 to Page conversion."""

        output_path.parent.mkdir(parents=True, exist_ok=True)
        parent = input_path.parent
        stem = input_path.stem
        filetype = input_path.suffix.split(".")[1]
        md_path = self._to_markdown(input_path, output_path)
        with open(md_path, "r") as md_file:
            md_content = md_file.read()
        metadata = parent / (stem+"_metadata.yml")
        with open(metadata, "r") as metadata_file:
            metadata_content = yaml.safe_load(metadata_file)
        url = metadata_content["URL"]
        timestamp = [i[1] for i in self.paragraphs]
        page = VidPage(pagename=stem,content={"text": md_content, "timestamp": timestamp}, filetype=filetype, page_url=url)

        return page

converter = VideoConverter()
converter._to_page(Path("/home/bot/roarai/rag/scraper/Scraper_master/Denero_videos/Self-Reference/Self-Reference.mp4"), Path("/home/bot/roarai/rag/scraper/Scraper_master/test"))