from pathlib import Path
import time
import whisper
from rag.file_conversion_router.conversion.base_converter import BaseConverter


class VideoConverter(BaseConverter):
    def __init__(self):
        super().__init__()

    # Override
    import whisper
    import time

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

    def convert_mp4_to_wav(mp4_path, wav_path):
        audio_clip = AudioFileClip(mp4_path)  # Load the audio track from the MP4 file
        audio_clip.write_audiofile(wav_path)  # Save the audio as a WAV file
        audio_clip.close()  # Close the clip to free resources
    def _to_page(self, input_path: Path, output_path: Path, url) -> Page:
        """Perform mp4 to Page conversion."""
        wav_filename = os.path.join(video_path, os.path.splitext(os.path.basename(download_filename))[0] + '.wav')

        output_path.parent.mkdir(parents=True, exist_ok=True)
        # parent = input_path.parent
        stem = input_path.stem
        filetype = input_path.suffix.split(".")[1]
        transcript = self.transcribe_audio_with_whisper(input_path)
        video =