import whisperx
from moviepy import AudioFileClip
from scenedetect import open_video, SceneManager
from scenedetect.detectors import AdaptiveDetector
from scenedetect.scene_manager import save_images, write_scene_list
import os
from pathlib import Path
from rag.file_conversion_router.conversion.base_converter import BaseConverter
from transformers import pipeline, set_seed, AutoTokenizer, AutoModelForCausalLM
import torch
import json
import re

class VideoConverter(BaseConverter):
    model_id = "meta-llama/Llama-3.1-8B-Instruct"
    # model_id = "THUDM/GLM-4-9B-0414"
    auto_tokenizer = None
    pipeline = None

    def __init__(self, course_name, course_id):
        super().__init__(self, course_name, course_id)
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

    def title_with_chat_completion(self, text, input_video_name):
        VideoConverter.initialize_static_resources()
        terminators = [
            VideoConverter.pipeline.tokenizer.eos_token_id,
            VideoConverter.pipeline.tokenizer.convert_tokens_to_ids("<|eot_id|>")
        ]
        messages = [
            {"role": "system",
             "content": "You are a helpful assistant."},
            {"role": "user",
             "content": f"Generate a short summary that serve as a title for the following transcript of a part of the lecture video named {input_video_name}. Do not over think of the meaning of the paragraph. Provide the title directly with quotation mark and \"Title:\", e.g. Title: \"Application of recursion\" \n---\n {text}"}
        ]
        prompt = VideoConverter.pipeline.tokenizer.apply_chat_template(
            messages,
            tokenize=False,
            add_generation_prompt=True
        )
        outputs = VideoConverter.pipeline(
            prompt,
            max_new_tokens=4096,  # Much shorter since we just want a title
            # eos_token_id=terminators,
            do_sample=False,  # Try with do_sample=False for more deterministic results
            temperature=0.3,  # Lower temperature for less randomness
        )
        title = outputs[0]["generated_text"][len(prompt):].strip().split('"')[1]
        # title = outputs[0]["generated_text"].split('</think>')[1].strip().split('"')[1]
        return title

    def convert_mp4_to_wav(self, mp4_file_path):
        wav_file_path = mp4_file_path.with_suffix(".wav")
        if not wav_file_path.exists():
            print(mp4_file_path)
            audio_clip = AudioFileClip(str(mp4_file_path))  # Load the audio track from the MP4 file
            audio_clip.write_audiofile(str(wav_file_path))  # Save the audio as a WAV file
            audio_clip.close()  # Close the clip to free resources
        return wav_file_path

    def _video_convert_whisperx(self, audio_file_path):
        device = "cuda"
        batch_size = 16
        compute_type = "float16"
        model = whisperx.load_model('large-v3-turbo', device='cuda', compute_type=compute_type)
        audio = whisperx.load_audio(audio_file_path)
        result = model.transcribe(audio, batch_size=batch_size)
        model_a, metadata = whisperx.load_align_model(language_code=result["language"], device=device)
        result = whisperx.align(result["segments"], model_a, metadata, audio, device, return_char_alignments=False)
        diarize_model = whisperx.DiarizationPipeline(use_auth_token=500, device=device)
        diarize_segments = diarize_model(audio)
        result = whisperx.assign_word_speakers(diarize_segments, result)
        segments = []
        if "segments" in result:
            for segment in result["segments"]:
                segment_dict = {
                    "start": segment["start"],
                    "end": segment["end"],
                    "text": segment["text"],
                    "speaker": segment["speaker"] if "speaker" in segment else "UNKNOWN"
                }
                segments.append(segment_dict)
        print(segments)
        import gc
        gc.collect()
        torch.cuda.empty_cache()
        del model
        return segments

    def paragraph_generator(self, transcript, seg_time):
        paragraphs = []
        if not seg_time:
            return [([chunk['text'] for chunk in transcript], 0)]
        for seg_start, seg_end in seg_time:
            paragraph = [chunk['text'] for chunk in transcript if seg_start <= chunk['start'] <= seg_end]
            paragraphs.append((paragraph, seg_start))
        self.paragraphs = paragraphs
        return paragraphs

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
                num_images=1,
                output_dir=images_output_dir,
            )
            scene_times = [(start_time.get_seconds(), end_time.get_seconds()) for start_time, end_time in scene_list]

            for i, ((start_time, end_time), images) in enumerate(zip(scene_list, image_filenames.values()), start=1):
                print(f"Scene {i}: Start Time: {start_time.get_seconds()}, End Time: {end_time.get_seconds()}")
                for image_path in images:
                    print(f"  - {image_path}")
            return scene_times

        video, scene_manager, images_output_dir = setup_scene_detection(str(video_path))
        scene_times = detect_scenes_and_save_images(video, scene_manager, images_output_dir)
        return scene_times



    def get_speaker_json(self, input_path, segments, output_path):
        self.get_timestamp_and_speaker(segments, output_path)

    def get_timestamp_and_speaker(self, segments, output_path):
        segments_formatted = []
        for segment in segments:
            # Extract the necessary information with fallback defaults.
            start = segment.get("start", 0)
            end = segment.get("end", 0)
            text = segment.get("text", "")
            speaker = segment.get("speaker", "UNKNOWN")

            # Convert time from seconds (float) to HH:MM:SS.mmm formatted string.
            formatted_start = self.format_timestamp(start)
            formatted_end = self.format_timestamp(end)
            segment_dict = {
                "start time": formatted_start,
                "end time": formatted_end,
                "speaker": speaker,
                "text content": text
            }
            segments_formatted.append(segment_dict)

            # Write the list of dictionaries to the output file as JSON.
        with open(output_path, 'w', encoding='utf-8') as outfile:
            json.dump(segments_formatted, outfile, indent=4)

    def format_timestamp(self, seconds):
        """Convert seconds (float) to a timestamp string HH:MM:SS.mmm"""
        hours = int(seconds // 3600)
        minutes = int((seconds % 3600) // 60)
        seconds_remainder = seconds % 60
        return f"{hours:02d}:{minutes:02d}:{seconds_remainder:06.3f}"

    def paragraph_generator(self, transcript, seg_time):
        paragraphs = []
        if not seg_time:
            return [([chunk['text'] for chunk in transcript], 0)]
        for seg_start, seg_end in seg_time:
            paragraph = [chunk['text'] for chunk in transcript if seg_start <= chunk['start'] <= seg_end]
            paragraphs.append((paragraph, seg_start))
        self.paragraphs = paragraphs
        return paragraphs

    def generate_summaries_of_titles(self, titles, input_video_name, chunk_size=2):
        VideoConverter.initialize_static_resources()
        summaries = []

        chunks = [titles[i:i + chunk_size] for i in range(0, len(titles), chunk_size)]

        for idx, chunk in enumerate(chunks):
            # If the last chunk has only one title, skip generating a summary for it
            if len(chunk) == 1 and idx == len(chunks) - 1:
                break
            titles_list = "\n".join(f"- {t}" for t in chunk)
            messages = [
                {"role": "system", "content": "You are a helpful assistant that creates concise summaries."},
                {"role": "user", "content": (
                    f"Generate a brief summary as a title that captures the main topics covered in this lecture video named "
                    f"{input_video_name} based on these section titles:\n{titles_list}\n\n"
                    "Provide your answer only in the following format (including the quotes):\n"
                    "Title: \"<summary>\""
                )}
            ]

            prompt = VideoConverter.pipeline.tokenizer.apply_chat_template(
                messages, tokenize=False, add_generation_prompt=True
            )

            outputs = VideoConverter.pipeline(
                prompt,
                max_new_tokens=100,
                do_sample=False,
                temperature=0.3,
            )

            first_output = outputs[0].get('generated_text', '')
            part = first_output.split('Title: ')[-1].strip()
            summary = part.strip('"')
            summaries.append(summary)

        # Clean up GPU cache
        import gc
        gc.collect()
        import torch
        torch.cuda.empty_cache()

        return summaries

    def _to_markdown(self, input_path, output_path):
        input_video_name = os.path.basename(input_path)
        audio = self.convert_mp4_to_wav(input_path)
        seg_time = self.process_video_scenes(input_path, output_path)
        transcript = self._video_convert_whisperx(str(audio))
        paragraphs = self.paragraph_generator(transcript, seg_time)

        titles = []
        paragraph_texts = []

        for i, (paragraph, time) in enumerate(paragraphs):
            paragraph_text = ' '.join(paragraph)
            if paragraph_text:
                title = self.title_with_chat_completion(paragraph_text, input_video_name)
                titles.append(title)
                paragraph_texts.append((title, paragraph_text))
            section_summaries = self.generate_summaries_of_titles(
                titles, input_video_name, 2
            )

            # 3. Build markdown content
            lines = []
            for idx, section_title in enumerate(section_summaries):
                lines.append(f"# {section_title}\n")
                start = idx * 2
                end = start + 2
                for title, text in paragraph_texts[start:end]:
                    lines.append(f"## {title}\n")
                    lines.append(f"{text}\n")

            markdown_content = "\n".join(lines)

        md_path = output_path.with_suffix(".md")

        with open(md_path, 'w') as md_file:
            md_file.write(markdown_content)
        # json_path = output_path.with_suffix(".json")
        # self.get_speaker_json(input_path, transcript, json_path)
        return md_path

if __name__ == "__main__":
    video_converter = VideoConverter()
    input_path = Path("/home/bot/bot/yk/language/video/A Polyglot's Daily Linguistic Workout.mp4")
    output_path = Path("/home/bot/bot/tai/rag/file_conversion_router/oput/A Polyglot's Daily Linguistic Workout.md")
    video_converter._to_markdown(input_path, output_path)
