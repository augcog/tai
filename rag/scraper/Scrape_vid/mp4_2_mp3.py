from moviepy.editor import *

# Load the mp4 file
video = VideoFileClip("test.mp4")

# Shorten the video to a 40-second clip from the start
short_video = video.subclip(0, 40)

# Extract audio from the shortened video
audio = short_video.audio
audio.write_audiofile("test.mp3")