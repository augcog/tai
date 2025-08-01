import yt_dlp

# Test with cookies file
# ydl_opts = {
#     'cookiefile': '/home/bot/bot/yk/YK_final/www.youtube.com_cookies.txt',
#     'quiet': True,
#     "noplaylist": True
# }

# ydl_opts = {
#             "cookiefile": "/home/bot/bot/yk/YK_final/www.youtube.com_cookies.txt",
#             # "outtmpl": os.path.join(folder, "%(title)s.%(ext)s"),
#             "format": "best",
#             "quiet": True,
#             "noplaylist": False,
#         }
#
# with yt_dlp.YoutubeDL(ydl_opts) as ydl:
#     try:
#         ydl.download(['https://www.youtube.com/watch?v=pveIuZT0GJE&list=PL6BsET-8jgYXvcnnEX7x2_USaYug9xZFv'])
#     except Exception as e:
#         print("Cookies failed:", e)
url='https://www.youtube.com/playlist?list=PLx38hZJ5RLZenHEh7hvBd76Dpsm7BhrRK'
# url='https://www.youtube.com/watch?v=4BH-JJ1eAOk'
ydl_opts = {"quiet": True,
            "no_warnings": True,
            "noplaylist": False,
            "cookiefile": "/home/bot/bot/yk/YK_final/www.youtube.com_cookies.txt",
            "format": "best",
            "outtmpl": "./%(playlist_title&{}|)s/%(playlist_index&{}-|)s%(title)s.%(ext)s",
            "ignoreerrors": True}
# "outtmpl": "%(playlist_title)s/%(playlist_index)s-%(title)s.%(ext)s"
# "outtmpl": "%(playlist_title&{}/|)s%(playlist_index&{}-|)s%(title)s.%(ext)s"

# '[CS 61A SU24] Discussion 01/1-Race.mp4'
# '[CS 61A SU24] Discussion 01⧸1-Race.mp4'
with yt_dlp.YoutubeDL(ydl_opts) as ydl:
    a=ydl.extract_info(url, download=True)
    print(a)
