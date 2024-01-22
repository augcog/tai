
## Goal

* In this tutorial, you will learn how to convert images from one color-space to another, like BGR \(\leftrightarrow\) Gray, BGR \(\leftrightarrow\) HSV, etc.
* In addition to that, we will create an application to extract a colored object in a video
* You will learn the following functions: **[cv.cvtColor()](../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab "Converts an image from one color space to another. ")**, **[cv.inRange()](../../d2/de8/group__core__array.html#ga48af0ab51e36436c5d04340e036ce981 "Checks if array elements lie between the elements of two other arrays. ")**, etc.

## Changing Color-space

There are more than 150 color-space conversion methods available in OpenCV. But we will look into only two, which are most widely used ones: BGR \(\leftrightarrow\) Gray and BGR \(\leftrightarrow\) HSV.

For color conversion, we use the function cv.cvtColor(input\_image, flag) where flag determines the type of conversion.

For BGR \(\rightarrow\) Gray conversion, we use the flag [cv.COLOR\_BGR2GRAY](../../d8/d01/group__imgproc__color__conversions.html#gga4e0972be5de079fed4e3a10e24ef5ef0a353a4b8db9040165db4dacb5bcefb6ea "convert between RGB/BGR and grayscale, color conversions "). Similarly for BGR \(\rightarrow\) HSV, we use the flag [cv.COLOR\_BGR2HSV](../../d8/d01/group__imgproc__color__conversions.html#gga4e0972be5de079fed4e3a10e24ef5ef0aa4a7f0ecf2e94150699e48c79139ee12 "convert RGB/BGR to HSV (hue saturation value) with H range 0..180 if 8 bit image, color conversions ..."). To get other flags, just run following commands in your Python terminal: 

>>> import cv2 as cv>>> flags = [i for i in dir(cv) if i.startswith('COLOR\_')]>>> [print](../../df/d57/namespacecv_1_1dnn.html#a43417dcaeb3c1e2a09b9d948e234c366 "../../df/d57/namespacecv_1_1dnn.html#a43417dcaeb3c1e2a09b9d948e234c366")( flags ) NoteFor HSV, hue range is [0,179], saturation range is [0,255], and value range is [0,255]. Different software use different scales. So if you are comparing OpenCV values with them, you need to normalize these ranges.
## Object Tracking

Now that we know how to convert a BGR image to HSV, we can use this to extract a colored object. In HSV, it is easier to represent a color than in BGR color-space. In our application, we will try to extract a blue colored object. So here is the method:

* Take each frame of the video
* Convert from BGR to HSV color-space
* We threshold the HSV image for a range of blue color
* Now extract the blue object alone, we can do whatever we want on that image.

Below is the code which is commented in detail: 

import cv2 as cvimport numpy as npcap = [cv.VideoCapture](../../d8/dfe/classcv_1_1VideoCapture.html "../../d8/dfe/classcv_1_1VideoCapture.html")(0)while(1): # Take each frame \_, frame = cap.read() # Convert BGR to HSV hsv = [cv.cvtColor](../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab "../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab")(frame, cv.COLOR\_BGR2HSV) # define range of blue color in HSV lower\_blue = np.array([110,50,50]) upper\_blue = np.array([130,255,255]) # Threshold the HSV image to get only blue colors mask = [cv.inRange](../../d2/de8/group__core__array.html#ga48af0ab51e36436c5d04340e036ce981 "../../d2/de8/group__core__array.html#ga48af0ab51e36436c5d04340e036ce981")(hsv, lower\_blue, upper\_blue) # Bitwise-AND mask and original image res = [cv.bitwise\_and](../../d2/de8/group__core__array.html#ga60b4d04b251ba5eb1392c34425497e14 "../../d2/de8/group__core__array.html#ga60b4d04b251ba5eb1392c34425497e14")(frame,frame, mask= mask) [cv.imshow](../../df/d24/group__highgui__opengl.html#gaae7e90aa3415c68dba22a5ff2cefc25d "../../df/d24/group__highgui__opengl.html#gaae7e90aa3415c68dba22a5ff2cefc25d")('frame',frame) [cv.imshow](../../df/d24/group__highgui__opengl.html#gaae7e90aa3415c68dba22a5ff2cefc25d "../../df/d24/group__highgui__opengl.html#gaae7e90aa3415c68dba22a5ff2cefc25d")('mask',mask) [cv.imshow](../../df/d24/group__highgui__opengl.html#gaae7e90aa3415c68dba22a5ff2cefc25d "../../df/d24/group__highgui__opengl.html#gaae7e90aa3415c68dba22a5ff2cefc25d")('res',res) k = [cv.waitKey](../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7 "../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7")(5) & 0xFF if k == 27: break[cv.destroyAllWindows](../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481 "../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481")() Below image shows tracking of the blue object:

![frame.jpg](../../frame.jpg)

image
NoteThere is some noise in the image. We will see how to remove it in later chapters.

This is the simplest method in object tracking. Once you learn functions of contours, you can do plenty of things like find the centroid of an object and use it to track the object, draw diagrams just by moving your hand in front of a camera, and other fun stuff.
## How to find HSV values to track?

This is a common question found in [stackoverflow.com](http://www.stackoverflow.com "http://www.stackoverflow.com"). It is very simple and you can use the same function, [cv.cvtColor()](../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab "Converts an image from one color space to another. "). Instead of passing an image, you just pass the BGR values you want. For example, to find the HSV value of Green, try the following commands in a Python terminal: 

>>> green = np.uint8([[[0,255,0 ]]])>>> hsv\_green = [cv.cvtColor](../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab "../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab")(green,cv.COLOR\_BGR2HSV)>>> [print](../../df/d57/namespacecv_1_1dnn.html#a43417dcaeb3c1e2a09b9d948e234c366 "../../df/d57/namespacecv_1_1dnn.html#a43417dcaeb3c1e2a09b9d948e234c366")( hsv\_green )[[[ 60 255 255]]] Now you take [H-10, 100,100] and [H+10, 255, 255] as the lower bound and upper bound respectively. Apart from this method, you can use any image editing tools like GIMP or any online converters to find these values, but don't forget to adjust the HSV ranges.

## Additional Resources

## Exercises

1. Try to find a way to extract more than one colored object, for example, extract red, blue, and green objects simultaneously.

