from file_conversion_router.conversion.base_converter import BaseConverter
from moviepy import AudioFileClip
import whisperx
from scenedetect import AdaptiveDetector
from scenedetect import open_video, SceneManager
from scenedetect.scene_manager import save_images, write_scene_list
from scenedetect import FrameTimecode
from dotenv import load_dotenv
import os
from openai import OpenAI
import json
import re
from textwrap import dedent


class VideoConverter(BaseConverter):
    def __init__(self, course_name, course_id):
        super().__init__(course_id=course_id,course_name=course_name)
        self.section_titles = [dict]
        self.file_name = ""
        self.paragraphs = []
        self.index_helper = None

    def convert_mp4_to_wav(self, mp4_file_path,output_path):
        wav_file_path = output_path.with_suffix(".wav")
        if not wav_file_path.exists():
            audio_clip = AudioFileClip(str(mp4_file_path))  # Load the audio track from the MP4 file
            audio_clip.write_audiofile(str(wav_file_path))  # Save the audio as a WAV file
            audio_clip.close()  # Close the clip to free resources
        return wav_file_path

    def _video_convert_whisperx(self, audio_file_path):
        device = "cuda"
        batch_size = 16
        compute_type = "float16"
        model = whisperx.load_model('large-v3', device='cuda', compute_type=compute_type, language="en")
        audio = whisperx.load_audio(audio_file_path)
        result = model.transcribe(audio, batch_size=batch_size)
        model_a, metadata = whisperx.load_align_model(language_code="en", device=device)
        result = whisperx.align(result["segments"], model_a, metadata, audio, device, return_char_alignments=False)
        diarize_model = whisperx.DiarizationPipeline(use_auth_token=True, device=device)
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
        return segments

    def paragraph_generator(self, transcript, seg_time):
        """
        Generates a structured list of paragraphs. Each paragraph contains its
        start time and a list of utterances (speaker and their text).
        """
        structured_paragraphs = []
        if not seg_time:
            all_text = " ".join([chunk['text'].strip() for chunk in transcript])
            speaker = transcript[0].get('speaker', 'UNKNOWN') if transcript else 'UNKNOWN'
            return [{'start_time': 0, 'utterances': [{'speaker': speaker, 'text': all_text}]}]

        for seg_start, seg_end in seg_time:
            segments_in_range = [
                chunk for chunk in transcript
                if seg_start <= chunk['start'] and chunk['start'] <= seg_end
            ]

            if not segments_in_range:
                continue

            current_paragraph_utterances = []
            current_utterance_text = ""
            last_speaker = None

            for segment in segments_in_range:
                speaker = segment.get('speaker', 'UNKNOWN')
                cleaned_text = segment['text'].strip()

                if not cleaned_text:
                    continue

                # If a speaker is starting the paragraph or changing
                if speaker != last_speaker and last_speaker is not None:
                    # Save the previous speaker's full utterance
                    current_paragraph_utterances.append({
                        'speaker': last_speaker,
                        'text': current_utterance_text
                    })
                    current_utterance_text = ""  # Reset for new speaker

                # Append text to the current utterance
                if not current_utterance_text:
                    current_utterance_text = cleaned_text
                else:
                    current_utterance_text += " " + cleaned_text

                last_speaker = speaker

            # After the loop, add the last pending utterance
            if current_utterance_text:
                current_paragraph_utterances.append({
                    'speaker': last_speaker,
                    'text': current_utterance_text
                })

            # Add the completed paragraph structure to our list
            if current_paragraph_utterances:
                structured_paragraphs.append({
                    'start_time': seg_start,
                    'utterances': current_paragraph_utterances
                })

        self.paragraphs = structured_paragraphs
        self.generate_index_helper(structured_paragraphs)
        return structured_paragraphs

    def generate_index_helper(self, paragraphs):
        """
        Create index helper from paragraphs.
        Each paragraph is indexed by its start time.
        Returns a dict mapping paragraph_index to start_time.
        """
        self.index_helper = {}
        for i, paragraph in enumerate(paragraphs):
            start_time = paragraph['start_time']
            paragraph_index = i + 1  # 1-based index
            self.index_helper[paragraph_index] = start_time

    def write_to_markdown(self, paragraphs):
        """
        Writes the structured paragraphs to a Markdown file, ensuring speakers
        in the same paragraph appear on new lines, not in new paragraphs.
        """
        markdown_lines = []
        for i, paragraph in enumerate(paragraphs, 1):
            paragraph_lines = []
            for utterance in paragraph['utterances']:
                speaker = utterance['speaker']
                text = utterance['text'].strip()
                if text:
                    paragraph_lines.append(f"{speaker}: {text}")

            # Join all speakers in the same paragraph with single \n
            if paragraph_lines:
                markdown_lines.append("\n".join(paragraph_lines))

        # Join paragraphs with double \n to separate them
        return "\n\n".join(markdown_lines)

    def process_video_scenes(self, video_path, output_path):
        def setup_scene_detection(video_path, custom_window_width=50, custom_weights=None):
            video = open_video(video_path)
            scene_manager = SceneManager()
            output_folder = os.path.dirname(output_path)
            images_folder_name = os.path.splitext(os.path.basename(video_path))[0] + "_images"
            images_output_dir = os.path.join(output_folder, images_folder_name)
            os.makedirs(images_output_dir, exist_ok=True)

            if custom_weights is None:
                custom_weights = AdaptiveDetector.Components(
                    delta_hue=0.1,
                    delta_sat=1.0,
                    delta_lum=1.0,
                    delta_edges=1.0
                )

            # min_scene_len = 1000  # Minimum scene length in seconds
            min_scene_len = 25 * 30  # 150 frames
            # adaptive_threshold = 3.0  # Adaptive threshold for scene detection
            adaptive_threshold = 7.0

            adaptive_detector = AdaptiveDetector(
                window_width=custom_window_width,
                weights=custom_weights,
                min_scene_len=min_scene_len,
                adaptive_threshold=adaptive_threshold
            )

            scene_manager.add_detector(adaptive_detector)
            return video, scene_manager, images_output_dir

        def detect_scenes_and_save_images(video, scene_manager, images_output_dir):
            scene_manager.detect_scenes(video, show_progress=True)
            scene_list = scene_manager.get_scene_list()

            # Check if scene detection found any scenes
            if not scene_list or len(scene_list) == 0:
                print("No scenes detected. Creating fallback scene using entire video duration.")

                # Create a fallback scene that spans the entire video
                start_time = video.start_time if hasattr(video, 'start_time') else FrameTimecode('00:00:00.000',
                                                                                                 fps=video.frame_rate)
                end_time = video.duration if hasattr(video, 'duration') else FrameTimecode(
                    video.frame_count / video.frame_rate, fps=video.frame_rate)

                # Create scene list with single scene covering entire video
                scene_list = [(start_time, end_time)]
                print(f"Fallback scene created: {start_time} to {end_time}")

            # Save scene list to CSV
            csv_file_path = os.path.join(images_output_dir, "detected_scenes.csv")
            with open(csv_file_path, 'w', newline='') as csv_file:
                write_scene_list(csv_file, scene_list)
            print(f"Scene list saved to {csv_file_path}")

            # Save images for detected/fallback scenes
            try:
                image_filenames = save_images(
                    scene_list=scene_list,
                    video=video,
                    num_images=1,
                    output_dir=images_output_dir,
                )
            except Exception as e:
                print(f"Warning: Could not save images: {e}")
                image_filenames = {}

            # Convert scene times to seconds for return value
            scene_times = [(start_time.get_seconds(), end_time.get_seconds()) for start_time, end_time in scene_list]

            # Print scene information
            for i, ((start_time, end_time), images) in enumerate(
                    zip(scene_list, image_filenames.values() if image_filenames else [[] for _ in scene_list]),
                    start=1):
                print(
                    f"Scene {i}: Start Time: {start_time.get_seconds():.3f}s ({start_time}), End Time: {end_time.get_seconds():.3f}s ({end_time})")
                for image_path in images:
                    print(f"  - {image_path}")

            return scene_times

        # Main execution
        video, scene_manager, images_output_dir = setup_scene_detection(str(video_path))
        scene_times = detect_scenes_and_save_images(video, scene_manager, images_output_dir)
        print(f"Scene times: {scene_times}")
        return scene_times
    def get_speaker_json(self, segments, json_output_path):
        self.get_timestamp_and_speaker(segments, json_output_path)

    def get_timestamp_and_speaker(self, segments, json_output_path):
        segments_formatted = []
        for segment in segments:
            # Extract the necessary information with fallback defaults.
            start = segment.get("start", 0)
            end = segment.get("end", 0)
            text = segment.get("text", "")
            speaker = segment.get("speaker", "UNKNOWN")
            segment_dict = {
                "start time": start,
                "end time": end,
                "speaker": speaker,
                "text content": text
            }
            segments_formatted.append(segment_dict)

            # Write the list of dictionaries to the output file as JSON.
        with open(json_output_path, 'w', encoding='utf-8') as outfile:
            json.dump(segments_formatted, outfile, indent=4)

    def _to_markdown(self, input_path, output_path):
        self.file_name = input_path.name
        audio = self.convert_mp4_to_wav(input_path, output_path)
        seg_time = self.process_video_scenes(input_path, output_path)
        segments = self._video_convert_whisperx(str(audio))
        self.get_speaker_json(segments, output_path.with_suffix(".json"))
        paragraphs = self.paragraph_generator(segments, seg_time)
        md_path = output_path.with_suffix(".md")
        text = self.write_to_markdown(paragraphs)
        with open(md_path, "w", encoding="utf-8") as f:
            f.write(text)
        return md_path

    def update_index_helper(self, content_dict,md_content: str = None) -> None:
        """
        Update the index helper with titles and their corresponding times.
        This method assumes that content_dict contains a list of titles with their levels,
        and that self.index_helper is a dictionary mapping paragraph indices to their start times.
        The titles are expected to be in a structure like:
        {
            "titles_with_levels": [
                {"title": "Title 1", "level_of_title": "1"},
                {"title": "Title 2", "level_of_title": "2"},
                ...
            ]
        }

        The index_helper will be updated to map each title path (as a tuple of titles) to its corresponding start time.
        This is useful for quickly looking up the start time of a title in the video.
        """
        result: dict[tuple, float] = {}
        path_stack: list[str] = []

        for t in content_dict.get("titles_with_levels", []):
            title = t["title"].strip()
            level = int(t["level_of_title"])
            paragraph_idx = t['paragraph_index']

            # Get the time for this specific paragraph index
            current_time = self.index_helper.get(paragraph_idx, 0.0)

            # Update the path stack based on the level
            path_stack = path_stack[:level - 1]
            path_stack.append(title)
            path = tuple(path_stack)

            # Map the title path to its time
            result[path] = current_time

        # Replace the helper so callers can do quick look-ups
        self.index_helper = result
        self.add_line_number_to_index_helper(md_content)
