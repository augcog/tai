
# Goal

* Learn to handle mouse events in OpenCV
* You will learn these functions : **[cv.setMouseCallback()](../../d7/dfc/group__highgui.html#ga89e7806b0a616f6f1d502bd8c183ad3e "Sets mouse handler for the specified window.")**

# Simple Demo

Here, we create a simple application which draws a circle on an image wherever we double-click on it.

First we create a mouse callback function which is executed when a mouse event take place. Mouse event can be anything related to mouse like left-button down, left-button up, left-button double-click etc. It gives us the coordinates (x,y) for every mouse event. With this event and location, we can do whatever we like. To list all available events available, run the following code in Python terminal: 

import cv2 as cv
events = [i for i in dir(cv) if 'EVENT' in i]
print( events )
 Creating mouse callback function has a specific format which is same everywhere. It differs only in what the function does. So our mouse callback function does one thing, it draws a circle where we double-click. So see the code below. Code is self-explanatory from comments : 

import numpy as np
import cv2 as cv

# mouse callback function
def draw\_circle(event,x,y,flags,param):
 if event == cv.EVENT\_LBUTTONDBLCLK:
 [cv.circle](../../d6/d6e/group__imgproc__draw.html#gaf10604b069374903dbd0f0488cb43670 "../../d6/d6e/group__imgproc__draw.html#gaf10604b069374903dbd0f0488cb43670")(img,(x,y),100,(255,0,0),-1)

# Create a black image, a window and bind the function to window
img = np.zeros((512,512,3), np.uint8)
[cv.namedWindow](../../d7/dfc/group__highgui.html#ga5afdf8410934fd099df85c75b2e0888b "../../d7/dfc/group__highgui.html#ga5afdf8410934fd099df85c75b2e0888b")('image')
[cv.setMouseCallback](../../d7/dfc/group__highgui.html#ga89e7806b0a616f6f1d502bd8c183ad3e "../../d7/dfc/group__highgui.html#ga89e7806b0a616f6f1d502bd8c183ad3e")('image',draw\_circle)

while(1):
 [cv.imshow](../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563 "../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563")('image',img)
 if [cv.waitKey](../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7 "../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7")(20) & 0xFF == 27:
 break
[cv.destroyAllWindows](../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481 "../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481")()
[cv::imshow](../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563 "../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563")void imshow(const String &winname, InputArray mat)Displays an image in the specified window.
[cv::waitKey](../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7 "../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7")int waitKey(int delay=0)Waits for a pressed key.
[cv::namedWindow](../../d7/dfc/group__highgui.html#ga5afdf8410934fd099df85c75b2e0888b "../../d7/dfc/group__highgui.html#ga5afdf8410934fd099df85c75b2e0888b")void namedWindow(const String &winname, int flags=WINDOW\_AUTOSIZE)Creates a window.
[cv::destroyAllWindows](../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481 "../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481")void destroyAllWindows()Destroys all of the HighGUI windows.
[cv::setMouseCallback](../../d7/dfc/group__highgui.html#ga89e7806b0a616f6f1d502bd8c183ad3e "../../d7/dfc/group__highgui.html#ga89e7806b0a616f6f1d502bd8c183ad3e")void setMouseCallback(const String &winname, MouseCallback onMouse, void \*userdata=0)Sets mouse handler for the specified window.
[cv::circle](../../d6/d6e/group__imgproc__draw.html#gaf10604b069374903dbd0f0488cb43670 "../../d6/d6e/group__imgproc__draw.html#gaf10604b069374903dbd0f0488cb43670")void circle(InputOutputArray img, Point center, int radius, const Scalar &color, int thickness=1, int lineType=LINE\_8, int shift=0)Draws a circle.
 # More Advanced Demo

Now we go for a much better application. In this, we draw either rectangles or circles (depending on the mode we select) by dragging the mouse like we do in Paint application. So our mouse callback function has two parts, one to draw rectangle and other to draw the circles. This specific example will be really helpful in creating and understanding some interactive applications like object tracking, image segmentation etc. 

import numpy as np
import cv2 as cv

drawing = False # true if mouse is pressed
mode = True # if True, draw rectangle. Press 'm' to toggle to curve
ix,iy = -1,-1

# mouse callback function
def draw\_circle(event,x,y,flags,param):
 global ix,iy,drawing,mode

 if event == cv.EVENT\_LBUTTONDOWN:
 drawing = True
 ix,iy = x,y

 elif event == cv.EVENT\_MOUSEMOVE:
 if drawing == True:
 if mode == True:
 [cv.rectangle](../../d6/d6e/group__imgproc__draw.html#ga07d2f74cadcf8e305e810ce8eed13bc9 "../../d6/d6e/group__imgproc__draw.html#ga07d2f74cadcf8e305e810ce8eed13bc9")(img,(ix,iy),(x,y),(0,255,0),-1)
 else:
 [cv.circle](../../d6/d6e/group__imgproc__draw.html#gaf10604b069374903dbd0f0488cb43670 "../../d6/d6e/group__imgproc__draw.html#gaf10604b069374903dbd0f0488cb43670")(img,(x,y),5,(0,0,255),-1)

 elif event == cv.EVENT\_LBUTTONUP:
 drawing = False
 if mode == True:
 [cv.rectangle](../../d6/d6e/group__imgproc__draw.html#ga07d2f74cadcf8e305e810ce8eed13bc9 "../../d6/d6e/group__imgproc__draw.html#ga07d2f74cadcf8e305e810ce8eed13bc9")(img,(ix,iy),(x,y),(0,255,0),-1)
 else:
 [cv.circle](../../d6/d6e/group__imgproc__draw.html#gaf10604b069374903dbd0f0488cb43670 "../../d6/d6e/group__imgproc__draw.html#gaf10604b069374903dbd0f0488cb43670")(img,(x,y),5,(0,0,255),-1)
[cv::rectangle](../../d6/d6e/group__imgproc__draw.html#ga07d2f74cadcf8e305e810ce8eed13bc9 "../../d6/d6e/group__imgproc__draw.html#ga07d2f74cadcf8e305e810ce8eed13bc9")void rectangle(InputOutputArray img, Point pt1, Point pt2, const Scalar &color, int thickness=1, int lineType=LINE\_8, int shift=0)Draws a simple, thick, or filled up-right rectangle.
 Next we have to bind this mouse callback function to OpenCV window. In the main loop, we should set a keyboard binding for key 'm' to toggle between rectangle and circle. 

img = np.zeros((512,512,3), np.uint8)
[cv.namedWindow](../../d7/dfc/group__highgui.html#ga5afdf8410934fd099df85c75b2e0888b "../../d7/dfc/group__highgui.html#ga5afdf8410934fd099df85c75b2e0888b")('image')
[cv.setMouseCallback](../../d7/dfc/group__highgui.html#ga89e7806b0a616f6f1d502bd8c183ad3e "../../d7/dfc/group__highgui.html#ga89e7806b0a616f6f1d502bd8c183ad3e")('image',draw\_circle)

while(1):
 [cv.imshow](../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563 "../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563")('image',img)
 k = [cv.waitKey](../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7 "../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7")(1) & 0xFF
 if k == ord('m'):
 mode = not mode
 elif k == 27:
 break

[cv.destroyAllWindows](../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481 "../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481")()
 # Additional Resources

# Exercises

1. In our last example, we drew filled rectangle. You modify the code to draw an unfilled rectangle.

