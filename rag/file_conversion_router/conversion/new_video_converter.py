from file_conversion_router.conversion.base_converter import BaseConverter
from moviepy import AudioFileClip
import whisper
from pyannote.audio import Pipeline
from scenedetect import AdaptiveDetector
from scenedetect import open_video, SceneManager
from scenedetect.scene_manager import save_images, write_scene_list
from scenedetect import FrameTimecode
from dotenv import load_dotenv
import os
import json
import re
from pathlib import Path
import torch
from typing import List, Dict, Tuple
import warnings

warnings.filterwarnings("ignore", category=UserWarning, module="torchaudio._backend.utils")


class NewVideoConverter(BaseConverter):
    """
    Video converter using Whisper Large V3 for transcription and
    pyannote/speaker-diarization-community-1 for speaker diarization.
    """

    def __init__(self, course_name, course_code, file_uuid: str = None):
        super().__init__(course_code=course_code, course_name=course_name, file_uuid=file_uuid)
        self.section_titles = [dict]
        self.file_name = ""
        self.paragraphs = []
        self.index_helper = None

    def convert_mp4_to_wav(self, mp4_file_path, output_path):
        """Convert video file to WAV audio format."""
        wav_file_path = output_path.with_suffix(".wav")
        if not wav_file_path.exists():
            audio_clip = AudioFileClip(str(mp4_file_path))
            audio_clip.write_audiofile(str(wav_file_path))
            audio_clip.close()
        return wav_file_path

    def _video_convert_whisper_pyannote(self, audio_file_path: str) -> List[Dict]:
        """
        Transcribe audio using Whisper large-v3 and perform speaker diarization
        using pyannote.

        Args:
            audio_file_path: Path to the audio file

        Returns:
            List of segments with start, end, text, and speaker information
        """
        device = "cuda" if torch.cuda.is_available() else "cpu"

        # Step 1: Load Whisper large-v3 model
        print("Loading Whisper large-v3 model...")
        whisper_model = whisper.load_model("large-v3", device=device)

        # Step 2: Transcribe audio with word-level timestamps
        print("Transcribing audio...")
        whisper_result = whisper_model.transcribe(
            audio_file_path,
            language="en",
            word_timestamps=True,
            verbose=True
        )

        print("Loading pyannote speaker diarization model...")
        diarization_pipeline = Pipeline.from_pretrained(
            "pyannote/speaker-diarization-community-1",
            token=os.getenv("HUGGINGFACE_ACCESS_TOKEN")
)

        # Move to appropriate device
        if device == "cuda":
            diarization_pipeline.to(torch.device("cuda"))

        # Step 4: Perform speaker diarization
        print("Performing speaker diarization...")
        diarization = diarization_pipeline(audio_file_path)

        # Step 5: Align transcription with speaker diarization
        segments = self._align_transcription_with_speakers(
            whisper_result["segments"],
            diarization
        )

        return segments

    def _align_transcription_with_speakers(
        self,
        whisper_segments: List[Dict],
        diarization
    ) -> List[Dict]:
        """
        Align Whisper transcription segments with pyannote speaker diarization.

        Args:
            whisper_segments: Segments from Whisper transcription
            diarization: Speaker diarization result from pyannote

        Returns:
            List of aligned segments with speaker labels
        """
        aligned_segments = []

        # Create a mapping of time ranges to speakers
        speaker_timeline = []
        for turn, speaker in diarization.speaker_diarization:
            speaker_timeline.append({
                "start": turn.start,
                "end": turn.end,
                "speaker": speaker
            })

        # Align each Whisper segment with the most overlapping speaker
        for segment in whisper_segments:
            seg_start = segment["start"]
            seg_end = segment["end"]
            seg_text = segment["text"]

            # Find the speaker with the most overlap
            best_speaker = "UNKNOWN"
            max_overlap = 0.0

            for speaker_seg in speaker_timeline:
                # Calculate overlap
                overlap_start = max(seg_start, speaker_seg["start"])
                overlap_end = min(seg_end, speaker_seg["end"])
                overlap_duration = max(0, overlap_end - overlap_start)

                if overlap_duration > max_overlap:
                    max_overlap = overlap_duration
                    best_speaker = speaker_seg["speaker"]

            # Create aligned segment
            aligned_segment = {
                "start": seg_start,
                "end": seg_end,
                "text": seg_text,
                "speaker": best_speaker
            }
            aligned_segments.append(aligned_segment)

        return aligned_segments

    def paragraph_generator(self, transcript: List[Dict], seg_time: List[Tuple]) -> List[Dict]:
        """
        Generate structured paragraphs from transcript, grouped by scene times.
        Each paragraph contains its start time and a list of utterances (speaker and text).

        Args:
            transcript: List of segments with start, end, text, and speaker
            seg_time: List of (start, end) tuples representing scene boundaries

        Returns:
            List of structured paragraphs
        """
        structured_paragraphs = []

        if not seg_time:
            # No scene detection, create one paragraph with all content
            all_text = " ".join([chunk['text'].strip() for chunk in transcript])
            speaker = transcript[0].get('speaker', 'UNKNOWN') if transcript else 'UNKNOWN'
            return [{'start_time': 0, 'utterances': [{'speaker': speaker, 'text': all_text}]}]

        for seg_start, seg_end in seg_time:
            # Get all segments within this scene
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

                # If speaker changes, save previous speaker's utterance
                if speaker != last_speaker and last_speaker is not None:
                    current_paragraph_utterances.append({
                        'speaker': last_speaker,
                        'text': current_utterance_text
                    })
                    current_utterance_text = ""

                # Append text to current utterance
                if not current_utterance_text:
                    current_utterance_text = cleaned_text
                else:
                    current_utterance_text += " " + cleaned_text

                last_speaker = speaker

            # Add the last utterance
            if current_utterance_text:
                current_paragraph_utterances.append({
                    'speaker': last_speaker,
                    'text': current_utterance_text
                })

            # Add completed paragraph
            if current_paragraph_utterances:
                structured_paragraphs.append({
                    'start_time': seg_start,
                    'utterances': current_paragraph_utterances
                })

        self.paragraphs = structured_paragraphs
        self.generate_index_helper(structured_paragraphs)
        return structured_paragraphs

    def generate_index_helper(self, paragraphs: List[Dict]):
        """
        Create index helper from paragraphs.
        Each paragraph is indexed by its start time.
        """
        self.index_helper = {}
        for i, paragraph in enumerate(paragraphs):
            start_time = paragraph['start_time']
            paragraph_index = i + 1  # 1-based index
            self.index_helper[paragraph_index] = start_time

    def write_to_markdown(self, paragraphs: List[Dict]) -> str:
        """
        Convert structured paragraphs to Markdown format.
        Speakers in the same paragraph appear on new lines within the paragraph.

        Args:
            paragraphs: List of structured paragraphs

        Returns:
            Markdown formatted string
        """
        markdown_lines = []

        for i, paragraph in enumerate(paragraphs, 1):
            paragraph_lines = []
            for utterance in paragraph['utterances']:
                speaker = utterance['speaker']
                text = utterance['text'].strip()
                if text:
                    paragraph_lines.append(f"{speaker}: {text}")

            # Join speakers in the same paragraph with single newline
            if paragraph_lines:
                markdown_lines.append("\n".join(paragraph_lines))

        # Join paragraphs with double newline
        return "\n\n".join(markdown_lines)

    def process_video_scenes(self, video_path, output_path) -> List[Tuple[float, float]]:
        """
        Detect scenes in video using adaptive detector.

        Args:
            video_path: Path to video file
            output_path: Path for output files

        Returns:
            List of (start_time, end_time) tuples in seconds
        """
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

            min_scene_len = 25 * 30  # 750 frames
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

            # Fallback if no scenes detected
            if not scene_list or len(scene_list) == 0:
                print("No scenes detected. Creating fallback scene using entire video duration.")

                start_time = video.start_time if hasattr(video, 'start_time') else FrameTimecode('00:00:00.000', fps=video.frame_rate)
                end_time = video.duration if hasattr(video, 'duration') else FrameTimecode(video.frame_count / video.frame_rate, fps=video.frame_rate)

                scene_list = [(start_time, end_time)]
                print(f"Fallback scene created: {start_time} to {end_time}")

            # Save scene list to CSV
            csv_file_path = os.path.join(images_output_dir, "detected_scenes.csv")
            with open(csv_file_path, 'w', newline='') as csv_file:
                write_scene_list(csv_file, scene_list)
            print(f"Scene list saved to {csv_file_path}")

            # Save images for scenes
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

            # Convert scene times to seconds
            scene_times = [(start_time.get_seconds(), end_time.get_seconds()) for start_time, end_time in scene_list]

            return scene_times

        # Main execution
        video, scene_manager, images_output_dir = setup_scene_detection(str(video_path))
        scene_times = detect_scenes_and_save_images(video, scene_manager, images_output_dir)
        print(f"Scene times: {scene_times}")
        return scene_times

    def get_speaker_json(self, segments: List[Dict], json_output_path):
        """Export segments with timestamps and speakers to JSON."""
        self.get_timestamp_and_speaker(segments, json_output_path)

    def get_timestamp_and_speaker(self, segments: List[Dict], json_output_path):
        """
        Format segments and write to JSON file.

        Args:
            segments: List of segments with timing and speaker info
            json_output_path: Output path for JSON file
        """
        segments_formatted = []
        for segment in segments:
            segment_dict = {
                "start time": segment.get("start", 0),
                "end time": segment.get("end", 0),
                "speaker": segment.get("speaker", "UNKNOWN"),
                "text content": segment.get("text", "")
            }
            segments_formatted.append(segment_dict)

        # Write to JSON file
        with open(json_output_path, 'w', encoding='utf-8') as outfile:
            json.dump(segments_formatted, outfile, indent=4)

    def _to_markdown(self, input_path, output_path):
        """
        Main conversion method orchestrating the entire pipeline.

        Args:
            input_path: Path to input video file
            output_path: Path for output markdown file

        Returns:
            Path to generated markdown file
        """
        self.file_name = input_path.name

        # Step 1: Convert video to audio
        print(f"Converting {input_path.name} to audio...")
        audio = self.convert_mp4_to_wav(input_path, output_path)

        # Step 2: Detect scenes
        print("Detecting scenes...")
        seg_time = self.process_video_scenes(input_path, output_path)

        # Step 3: Transcribe and diarize
        print("Transcribing and performing speaker diarization...")
        segments = self._video_convert_whisper_pyannote(str(audio))

        # Step 4: Export JSON with timestamps
        print("Exporting JSON...")
        self.get_speaker_json(segments, output_path.with_suffix(".json"))

        # Step 5: Generate paragraphs
        print("Generating paragraphs...")
        paragraphs = self.paragraph_generator(segments, seg_time)

        # Step 6: Write to markdown
        print("Writing to markdown...")
        md_path = output_path.with_suffix(".md")
        text = self.write_to_markdown(paragraphs)
        with open(md_path, "w", encoding="utf-8") as f:
            f.write(text)

        print(f"Conversion complete! Markdown saved to: {md_path}")
        return md_path

    def update_index_helper(self, content_dict, md_content: str = None) -> None:
        """
        Update the index helper with titles and their corresponding times.
        For video files without traditional titles, this maintains paragraph-based indexing.
        """
        result: dict[tuple, float] = {}
        path_stack: list[str] = []

        for t in content_dict.get("titles_with_levels", []):
            title = t["title"].strip()
            level = int(t["level_of_title"])
            paragraph_idx = t['paragraph_index']

            # Get the time for this specific paragraph index
            current_time = self.index_helper.get(paragraph_idx)

            # Update the path stack based on the level
            path_stack = path_stack[:level - 1]
            path_stack.append(title)
            path = tuple(path_stack)

            # Map the title path to its time
            result[path] = current_time

        self.index_helper = result
        self.add_line_number_to_index_helper(md_content)


if __name__ == "__main__":
    load_dotenv()
    HUGGINGFACE_ACCESS_TOKEN = os.getenv("HUGGINGFACE_ACCESS_TOKEN")
    converter = NewVideoConverter(course_name="Test Course", course_code="TEST101")
    input_path = Path("/home/bot/bot/yk/YK_final/test_folder/1-Midterm 1 Question 1 Walkthrough.mp4")
    output_path = Path("/home/bot/bot/yk/YK_final/test_folder/")
    converter._to_markdown(input_path, output_path)
