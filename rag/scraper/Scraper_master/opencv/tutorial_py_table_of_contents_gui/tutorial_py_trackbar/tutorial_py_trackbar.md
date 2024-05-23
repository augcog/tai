
# Goal

* Learn to bind trackbar to OpenCV windows
* You will learn these functions : **[cv.getTrackbarPos()](../../d7/dfc/group__highgui.html#ga122632e9e91b9ec06943472c55d9cda8 "Returns the trackbar position.")**, **[cv.createTrackbar()](../../d7/dfc/group__highgui.html#gaf78d2155d30b728fc413803745b67a9b "Creates a trackbar and attaches it to the specified window.")** etc.

# Code Demo

Here we will create a simple application which shows the color you specify. You have a window which shows the color and three trackbars to specify each of B,G,R colors. You slide the trackbar and correspondingly window color changes. By default, initial color will be set to Black.

For [cv.createTrackbar()](../../d7/dfc/group__highgui.html#gaf78d2155d30b728fc413803745b67a9b "Creates a trackbar and attaches it to the specified window.") function, first argument is the trackbar name, second one is the window name to which it is attached, third argument is the default value, fourth one is the maximum value and fifth one is the callback function which is executed every time trackbar value changes. The callback function always has a default argument which is the trackbar position. In our case, function does nothing, so we simply pass.

Another important application of trackbar is to use it as a button or switch. OpenCV, by default, doesn't have button functionality. So you can use trackbar to get such functionality. In our application, we have created one switch in which application works only if switch is ON, otherwise screen is always black. 

import numpy as np
import cv2 as cv

def nothing(x):
 pass

# Create a black image, a window
img = np.zeros((300,512,3), np.uint8)
[cv.namedWindow](../../d7/dfc/group__highgui.html#ga5afdf8410934fd099df85c75b2e0888b "../../d7/dfc/group__highgui.html#ga5afdf8410934fd099df85c75b2e0888b")('image')

# create trackbars for color change
[cv.createTrackbar](../../d7/dfc/group__highgui.html#gaf78d2155d30b728fc413803745b67a9b "../../d7/dfc/group__highgui.html#gaf78d2155d30b728fc413803745b67a9b")('R','image',0,255,nothing)

[cv.createTrackbar](../../d7/dfc/group__highgui.html#gaf78d2155d30b728fc413803745b67a9b "../../d7/dfc/group__highgui.html#gaf78d2155d30b728fc413803745b67a9b")('G','image',0,255,nothing)
[cv.createTrackbar](../../d7/dfc/group__highgui.html#gaf78d2155d30b728fc413803745b67a9b "../../d7/dfc/group__highgui.html#gaf78d2155d30b728fc413803745b67a9b")('B','image',0,255,nothing)

# create switch for ON/OFF functionality
switch = '0 : OFF \n1 : ON'
[cv.createTrackbar](../../d7/dfc/group__highgui.html#gaf78d2155d30b728fc413803745b67a9b "../../d7/dfc/group__highgui.html#gaf78d2155d30b728fc413803745b67a9b")(switch, 'image',0,1,nothing)

while(1):
 [cv.imshow](../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563 "../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563")('image',img)
 k = [cv.waitKey](../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7 "../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7")(1) & 0xFF
 if k == 27:
 break

 # get current positions of four trackbars
 r = [cv.getTrackbarPos](../../d7/dfc/group__highgui.html#ga122632e9e91b9ec06943472c55d9cda8 "../../d7/dfc/group__highgui.html#ga122632e9e91b9ec06943472c55d9cda8")('R','image')
 g = [cv.getTrackbarPos](../../d7/dfc/group__highgui.html#ga122632e9e91b9ec06943472c55d9cda8 "../../d7/dfc/group__highgui.html#ga122632e9e91b9ec06943472c55d9cda8")('G','image')
 b = [cv.getTrackbarPos](../../d7/dfc/group__highgui.html#ga122632e9e91b9ec06943472c55d9cda8 "../../d7/dfc/group__highgui.html#ga122632e9e91b9ec06943472c55d9cda8")('B','image')
 s = [cv.getTrackbarPos](../../d7/dfc/group__highgui.html#ga122632e9e91b9ec06943472c55d9cda8 "../../d7/dfc/group__highgui.html#ga122632e9e91b9ec06943472c55d9cda8")(switch,'image')

 if s == 0:
 img[:] = 0
 else:
 img[:] = [b,g,r]

[cv.destroyAllWindows](../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481 "../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481")()
[cv::getTrackbarPos](../../d7/dfc/group__highgui.html#ga122632e9e91b9ec06943472c55d9cda8 "../../d7/dfc/group__highgui.html#ga122632e9e91b9ec06943472c55d9cda8")int getTrackbarPos(const String &trackbarname, const String &winname)Returns the trackbar position.
[cv::imshow](../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563 "../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563")void imshow(const String &winname, InputArray mat)Displays an image in the specified window.
[cv::waitKey](../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7 "../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7")int waitKey(int delay=0)Waits for a pressed key.
[cv::namedWindow](../../d7/dfc/group__highgui.html#ga5afdf8410934fd099df85c75b2e0888b "../../d7/dfc/group__highgui.html#ga5afdf8410934fd099df85c75b2e0888b")void namedWindow(const String &winname, int flags=WINDOW\_AUTOSIZE)Creates a window.
[cv::destroyAllWindows](../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481 "../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481")void destroyAllWindows()Destroys all of the HighGUI windows.
[cv::createTrackbar](../../d7/dfc/group__highgui.html#gaf78d2155d30b728fc413803745b67a9b "../../d7/dfc/group__highgui.html#gaf78d2155d30b728fc413803745b67a9b")int createTrackbar(const String &trackbarname, const String &winname, int \*value, int count, TrackbarCallback onChange=0, void \*userdata=0)Creates a trackbar and attaches it to the specified window.
 The screenshot of the application looks like below :

![](../../trackbar_screenshot.jpg)

image
# Exercises

1. Create a Paint application with adjustable colors and brush radius using trackbars. For drawing, refer previous tutorial on mouse handling.

