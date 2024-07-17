# Scrape_vid
1) In order to use this folder you will need to find the playlist of the video on youtube.
2) Then go to the file `Youtube_playlist_to_video.py` and change these according to your needs:
```
base_path = 'Denero'
os.makedirs(base_path, exist_ok=True)
playlist_url = 'https://www.youtube.com/watch?v=31EDjrN1x5k&list=PL6BsET-8jgYUA8ryM_zeRA3H_RAMNBrN3&ab_channel=JohnDeNero'
video_urls = get_playlist_urls(playlist_url)
download_videos(video_urls, base_path)
segment = Segment(base_path, video_urls)
segment.process_video_audio()
```
  - `base_path` is the folder where you want to save the videos.
  - `playlist_url` is the url of the playlist.
3) Run the file `Youtube_playlist_to_video.py` and it will download the videos and process them.
4) The processed videos will be saved in the folder `base_path`.
5) For each video folder will expect to see.
   - A folder containing the frames of the chunks of the video.
   - A mp4 file.
   - A wav file.
   - A pkl file containing the required information for embedding. 