from pathlib import Path
import time
import whisper
from rag.file_conversion_router.conversion.base_converter import BaseConverter
from rag.file_conversion_router.classes.page import Page
from moviepy.editor import AudioFileClip
from scenedetect import open_video, SceneManager
from scenedetect.detectors import AdaptiveDetector
from scenedetect.scene_manager import save_images, write_scene_list
from transformers import pipeline, set_seed, AutoTokenizer, AutoModelForCausalLM
import torch
import os

class VideoConverter(BaseConverter):
    def __init__(self):
        super().__init__()
        self.model_id = "meta-llama/Meta-Llama-3-8B-Instruct"
        # Initialize the tokenizer and model only once when the classes is instantiated
        self.auto_tokenizer = AutoTokenizer.from_pretrained(self.model_id)
        self.pipeline = pipeline(
            'text-generation',
            model=self.model_id,
            model_kwargs={"torch_dtype": torch.bfloat16},
            device="cuda"
        )  # Assumes a GPU is available at device 0
    # Override
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

    def convert_mp4_to_wav(self, mp4_file_path):
        wav_file_path = mp4_file_path.with_suffix(".wav")
        audio_clip = AudioFileClip(mp4_file_path)  # Load the audio track from the MP4 file
        audio_clip.write_audiofile(wav_file_path)  # Save the audio as a WAV file
        audio_clip.close()  # Close the clip to free resources
        return wav_file_path

    def process_video_scenes(self, video_path):
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

        video, scene_manager, images_output_dir = setup_scene_detection(video_path)
        scene_times = detect_scenes_and_save_images(video, scene_manager, images_output_dir)
        return scene_times

    def paragraph_generator(self, transcript, seg_time):
        paragraphs = []
        if not seg_time:
            return [([chunk['text'] for chunk in transcript], 0)]
        for seg_start, seg_end in seg_time:
            paragraph = [chunk['text'] for chunk in transcript if seg_start <= chunk['start'] <= seg_end]
            paragraphs.append((paragraph, seg_start))
        return paragraphs

    def _to_markdown(self, paragraphs):
        markdown_content = ""
        for i, (paragraph, time) in enumerate(paragraphs):
            paragraph_text = ''.join(paragraph)
            if paragraph_text:
                title = self.title_with_chat_completion(paragraph_text)
                markdown_content += f'# {title}\n\n'
                markdown_content += f'{paragraph_text}\n\n'
        return markdown_content

    def _to_page(self, input_path: Path, output_path: Path) -> Page:
        """Perform mp4 to Page conversion."""
        video = input_path
        audio = self.convert_mp4_to_wav(input_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        stem = input_path.stem
        filetype = input_path.suffix.split(".")[1]
        seg_time = self.process_video_scenes(video)
        transcript = self.transcribe_audio_with_whisper(audio)
        paragraphs = self.paragraph_generator(transcript, seg_time)
        # to markdown
        md_path = os.path.join(output_path, f'{video.stem}_content.md')
        md_content = self._to_markdown(paragraphs)
        with open(md_path, 'w') as md_file:
            md_file.write(md_content)

        return Page(content={"text": md_content}, filetype=filetype, page_url=url)
