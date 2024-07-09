from pytube import Playlist, YouTube
from moviepy.editor import AudioFileClip
from segment import Segment
import os
<<<<<<< HEAD
def get_playlist_urls(playlist_url):
    playlist = Playlist(playlist_url)
    return list(playlist.video_urls)
=======


def get_playlist_urls(playlist_url):
    playlist = Playlist(playlist_url)
    # Retrieve videos and their titles
    videos_with_titles = [(video, video.title) for video in playlist.videos]

    # Sanitize titles and sort videos by these titles
    videos_with_titles.sort(key=lambda x: "".join(char for char in x[1] if char.isalnum() or char in " -_").strip())

    # Return only the URLs, now sorted by the sanitized title
    return [video.watch_url for video, _ in videos_with_titles]
>>>>>>> cdbc2f5496cabb88b8715e1213624541579ec1fc

def convert_mp4_to_wav(mp4_path, wav_path):
    audio_clip = AudioFileClip(mp4_path)  # Load the audio track from the MP4 file
    audio_clip.write_audiofile(wav_path)  # Save the audio as a WAV file
    audio_clip.close()  # Close the clip to free resources
def download_videos(video_urls, base_path):
    for url in video_urls:
        video = YouTube(url)
        stream = video.streams.get_highest_resolution()

        # Create a unique folder for each video based on its title
        safe_title = "".join(x for x in video.title if x.isalnum() or x in " -_").strip()
        video_path = os.path.join(base_path, safe_title)
        os.makedirs(video_path, exist_ok=True)

        download_filename = stream.download(output_path=video_path)

<<<<<<< HEAD
        # Construct WAV filename in the same unique folder
        wav_filename = os.path.join(video_path, os.path.splitext(os.path.basename(download_filename))[0] + '.wav')

        # Convert to AV
        convert_mp4_to_wav(download_filename, wav_filename)

base_path = 'Denero'
=======
        # # Construct WAV filename in the same unique folder
        # wav_filename = os.path.join(video_path, os.path.splitext(os.path.basename(download_filename))[0] + '.wav')
        #
        # # Convert to AV
        # convert_mp4_to_wav(download_filename, wav_filename)
def get_directories(path):
    return sorted([d for d in os.listdir(path) if os.path.isdir(os.path.join(path, d))])


base_path = 'Denero_videos'
>>>>>>> cdbc2f5496cabb88b8715e1213624541579ec1fc
os.makedirs(base_path, exist_ok=True)
playlist_url = 'https://www.youtube.com/watch?v=31EDjrN1x5k&list=PL6BsET-8jgYUA8ryM_zeRA3H_RAMNBrN3&ab_channel=JohnDeNero'
video_urls = get_playlist_urls(playlist_url)
download_videos(video_urls, base_path)
<<<<<<< HEAD
segment = Segment(base_path, video_urls)
segment.process_video_audio()
=======
# print(video_urls)
# print(get_directories(base_path))
# segment = Segment(base_path, video_urls)
# segment.process_video_audio_to_md()
# segment.process_video_audio()
>>>>>>> cdbc2f5496cabb88b8715e1213624541579ec1fc
