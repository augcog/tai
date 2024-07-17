import whisper
import time


def transcribe_audio_with_whisper(audio_file_path):
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

if __name__ == "__main__":
    # Example usage
    audio_file = "Denero/Mutual Recursion/Mutual Recursion.wav"
    segments = transcribe_audio_with_whisper(audio_file)
    for segment in segments:
        print(f"Start: {segment['start']}, End: {segment['end']}, Text: \"{segment['text']}\"")
