import whisper
import json
import time
# Load the Whisper model

start_time = time.time()
model = whisper.load_model("base")

# Transcribe the audio file
result = model.transcribe("lecture01.wav")
# result = model.transcribe("Expressions.wav")
# Print the entire result object
print(json.dumps(result, indent=4))

# If you just want to print the transcribed text
print(result["text"])

# If you want to print detailed information about segments, assuming it's structured similarly
# to the JSON you provided earlier, you would need to iterate over the segments if they exist
# Path for the output text file
file_path = "pure_text.txt"
with open(file_path, 'w') as file:
    file.write(result["text"])
# Writing the segments into the text file
file_path = 'transcribed_text.txt'
with open(file_path, 'w') as file:
    if "segments" in result:
        for segment in result["segments"]:
            start = segment["start"]
            end = segment["end"]
            text = segment["text"]
            file.write(f"{start}-{end}: {text}\n")

print(f"Time taken: {time.time()-start_time}")
