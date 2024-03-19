from moviepy.editor import *

# Load the mp4 file
video = VideoFileClip("Expressions.mp4")

# Shorten the video to a 40-second clip from the start
# Extract audio from the shortened video
audio = video.audio

# Write the audio file, attempting to set the sample rate
audio.write_audiofile("Expressions.wav", fps=16000)
