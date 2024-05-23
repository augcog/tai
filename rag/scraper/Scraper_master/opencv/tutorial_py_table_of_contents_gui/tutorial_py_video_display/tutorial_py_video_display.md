
# Goal

* Learn to read video, display video, and save video.
* Learn to capture video from a camera and display it.
* You will learn these functions : **[cv.VideoCapture()](../../d8/dfe/classcv_1_1VideoCapture.html "Class for video capturing from video files, image sequences or cameras.")**, **[cv.VideoWriter()](../../dd/d9e/classcv_1_1VideoWriter.html "Video writer class.")**

# Capture Video from Camera

Often, we have to capture live stream with a camera. OpenCV provides a very simple interface to do this. Let's capture a video from the camera (I am using the built-in webcam on my laptop), convert it into grayscale video and display it. Just a simple task to get started.

To capture a video, you need to create a **VideoCapture** object. Its argument can be either the device index or the name of a video file. A device index is just the number to specify which camera. Normally one camera will be connected (as in my case). So I simply pass 0 (or -1). You can select the second camera by passing 1 and so on. After that, you can capture frame-by-frame. But at the end, don't forget to release the capture. 

import numpy as np
import cv2 as cv

cap = [cv.VideoCapture](../../d8/dfe/classcv_1_1VideoCapture.html "../../d8/dfe/classcv_1_1VideoCapture.html")(0)
if not cap.isOpened():
 print("Cannot open camera")
 exit()
while True:
 # Capture frame-by-frame
 ret, frame = cap.read()

 # if frame is read correctly ret is True
 if not ret:
 print("Can't receive frame (stream end?). Exiting ...")
 break
 # Our operations on the frame come here
 gray = [cv.cvtColor](../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab "../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab")(frame, cv.COLOR\_BGR2GRAY)
 # Display the resulting frame
 [cv.imshow](../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563 "../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563")('frame', gray)
 if [cv.waitKey](../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7 "../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7")(1) == ord('q'):
 break

# When everything done, release the capture
cap.release()
[cv.destroyAllWindows](../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481 "../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481")()
[cv::VideoCapture](../../d8/dfe/classcv_1_1VideoCapture.html "../../d8/dfe/classcv_1_1VideoCapture.html")Class for video capturing from video files, image sequences or cameras.**Definition** videoio.hpp:731
[cv::imshow](../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563 "../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563")void imshow(const String &winname, InputArray mat)Displays an image in the specified window.
[cv::waitKey](../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7 "../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7")int waitKey(int delay=0)Waits for a pressed key.
[cv::destroyAllWindows](../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481 "../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481")void destroyAllWindows()Destroys all of the HighGUI windows.
[cv::cvtColor](../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab "../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab")void cvtColor(InputArray src, OutputArray dst, int code, int dstCn=0)Converts an image from one color space to another.
 `cap.read()` returns a bool (`True`/`False`). If the frame is read correctly, it will be `True`. So you can check for the end of the video by checking this returned value.

Sometimes, cap may not have initialized the capture. In that case, this code shows an error. You can check whether it is initialized or not by the method **cap.isOpened()**. If it is `True`, OK. Otherwise open it using **cap.open()**.

You can also access some of the features of this video using **cap.get(propId)** method where propId is a number from 0 to 18. Each number denotes a property of the video (if it is applicable to that video). Full details can be seen here: [cv::VideoCapture::get()](../../d8/dfe/classcv_1_1VideoCapture.html#aa6480e6972ef4c00d74814ec841a2939 "Returns the specified VideoCapture property."). Some of these values can be modified using **cap.set(propId, value)**. Value is the new value you want.

For example, I can check the frame width and height by `cap.get([cv.CAP_PROP_FRAME_WIDTH](../../d4/d15/group__videoio__flags__base.html#ggaeb8dd9c89c10a5c63c139bf7c4f5704dab26d2ba37086662261148e9fe93eecad "Width of the frames in the video stream."))` and `cap.get([cv.CAP_PROP_FRAME_HEIGHT](../../d4/d15/group__videoio__flags__base.html#ggaeb8dd9c89c10a5c63c139bf7c4f5704dad8b57083fd9bd58e0f94e68a54b42b7e "Height of the frames in the video stream."))`. It gives me 640x480 by default. But I want to modify it to 320x240. Just use `ret = cap.set([cv.CAP_PROP_FRAME_WIDTH](../../d4/d15/group__videoio__flags__base.html#ggaeb8dd9c89c10a5c63c139bf7c4f5704dab26d2ba37086662261148e9fe93eecad "Width of the frames in the video stream."),320)` and `ret = cap.set([cv.CAP_PROP_FRAME_HEIGHT](../../d4/d15/group__videoio__flags__base.html#ggaeb8dd9c89c10a5c63c139bf7c4f5704dad8b57083fd9bd58e0f94e68a54b42b7e "Height of the frames in the video stream."),240)`.

NoteIf you are getting an error, make sure your camera is working fine using any other camera application (like Cheese in Linux).
# Playing Video from file

Playing video from file is the same as capturing it from camera, just change the camera index to a video file name. Also while displaying the frame, use appropriate time for `[cv.waitKey()](../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7 "Waits for a pressed key.")`. If it is too less, video will be very fast and if it is too high, video will be slow (Well, that is how you can display videos in slow motion). 25 milliseconds will be OK in normal cases. 

import numpy as np
import cv2 as cv

cap = [cv.VideoCapture](../../d8/dfe/classcv_1_1VideoCapture.html "../../d8/dfe/classcv_1_1VideoCapture.html")('vtest.avi')

while cap.isOpened():
 ret, frame = cap.read()

 # if frame is read correctly ret is True
 if not ret:
 print("Can't receive frame (stream end?). Exiting ...")
 break
 gray = [cv.cvtColor](../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab "../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab")(frame, cv.COLOR\_BGR2GRAY)

 [cv.imshow](../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563 "../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563")('frame', gray)
 if [cv.waitKey](../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7 "../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7")(1) == ord('q'):
 break

cap.release()
[cv.destroyAllWindows](../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481 "../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481")()
NoteMake sure a proper version of ffmpeg or gstreamer is installed. Sometimes it is a headache to work with video capture, mostly due to wrong installation of ffmpeg/gstreamer.
# Saving a Video

So we capture a video and process it frame-by-frame, and we want to save that video. For images, it is very simple: just use `[cv.imwrite()](../../d4/da8/group__imgcodecs.html#ga8ac397bd09e48851665edbe12aa28f25 "Saves an image to a specified file.")`. Here, a little more work is required.

This time we create a **VideoWriter** object. We should specify the output file name (eg: output.avi). Then we should specify the **FourCC** code (details in next paragraph). Then number of frames per second (fps) and frame size should be passed. And the last one is the **isColor** flag. If it is `True`, the encoder expect color frame, otherwise it works with grayscale frame.

[FourCC](https://en.wikipedia.org/wiki/FourCC "https://en.wikipedia.org/wiki/FourCC") is a 4-byte code used to specify the video codec. The list of available codes can be found in [fourcc.org](https://fourcc.org/codecs.php "https://fourcc.org/codecs.php"). It is platform dependent. The following codecs work fine for me.

* In Fedora: DIVX, XVID, MJPG, X264, WMV1, WMV2. (XVID is more preferable. MJPG results in high size video. X264 gives very small size video)
* In Windows: DIVX (More to be tested and added)
* In OSX: MJPG (.mp4), DIVX (.avi), X264 (.mkv).

FourCC code is passed as ‘cv.VideoWriter\_fourcc('M’,'J','P','G')`or` cv.VideoWriter\_fourcc(\*'MJPG')` for MJPG.

The below code captures from a camera, flips every frame in the vertical direction, and saves the video. 

import numpy as np
import cv2 as cv

cap = [cv.VideoCapture](../../d8/dfe/classcv_1_1VideoCapture.html "../../d8/dfe/classcv_1_1VideoCapture.html")(0)

# Define the codec and create VideoWriter object
fourcc = cv.VideoWriter\_fourcc(\*'XVID')
out = [cv.VideoWriter](../../dd/d9e/classcv_1_1VideoWriter.html "../../dd/d9e/classcv_1_1VideoWriter.html")('output.avi', fourcc, 20.0, (640, 480))

while cap.isOpened():
 ret, frame = cap.read()
 if not ret:
 print("Can't receive frame (stream end?). Exiting ...")
 break
 frame = [cv.flip](../../d2/de8/group__core__array.html#gaca7be533e3dac7feb70fc60635adf441 "../../d2/de8/group__core__array.html#gaca7be533e3dac7feb70fc60635adf441")(frame, 0)

 # write the flipped frame
 out.write(frame)

 [cv.imshow](../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563 "../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563")('frame', frame)
 if [cv.waitKey](../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7 "../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7")(1) == ord('q'):
 break

# Release everything if job is finished
cap.release()
out.release()
[cv.destroyAllWindows](../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481 "../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481")()
[cv::VideoWriter](../../dd/d9e/classcv_1_1VideoWriter.html "../../dd/d9e/classcv_1_1VideoWriter.html")Video writer class.**Definition** videoio.hpp:1009
[cv::flip](../../d2/de8/group__core__array.html#gaca7be533e3dac7feb70fc60635adf441 "../../d2/de8/group__core__array.html#gaca7be533e3dac7feb70fc60635adf441")void flip(InputArray src, OutputArray dst, int flipCode)Flips a 2D array around vertical, horizontal, or both axes.
# Additional Resources

# Exercises

