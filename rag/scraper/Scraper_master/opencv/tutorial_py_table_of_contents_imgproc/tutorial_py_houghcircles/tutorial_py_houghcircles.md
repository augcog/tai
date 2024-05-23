
# Goal

In this chapter,

* We will learn to use Hough Transform to find circles in an image.
* We will see these functions: **[cv.HoughCircles()](../../dd/d1a/group__imgproc__feature.html#ga47849c3be0d0406ad3ca45db65a25d2d "Finds circles in a grayscale image using the Hough transform.")**

# Theory

A circle is represented mathematically as \((x-x\_{center})^2 + (y - y\_{center})^2 = r^2\) where \((x\_{center},y\_{center})\) is the center of the circle, and \(r\) is the radius of the circle. From equation, we can see we have 3 parameters, so we need a 3D accumulator for hough transform, which would be highly ineffective. So OpenCV uses more trickier method, **Hough Gradient Method** which uses the gradient information of edges.

The function we use here is **[cv.HoughCircles()](../../dd/d1a/group__imgproc__feature.html#ga47849c3be0d0406ad3ca45db65a25d2d "Finds circles in a grayscale image using the Hough transform.")**. It has plenty of arguments which are well explained in the documentation. So we directly go to the code. 

import numpy as np
import cv2 as cv

img = [cv.imread](../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8 "../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8")('opencv-logo-white.png', cv.IMREAD\_GRAYSCALE)
assert img is not None, "file could not be read, check with os.path.exists()"
img = [cv.medianBlur](../../d4/d86/group__imgproc__filter.html#ga564869aa33e58769b4469101aac458f9 "../../d4/d86/group__imgproc__filter.html#ga564869aa33e58769b4469101aac458f9")(img,5)
cimg = [cv.cvtColor](../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab "../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab")(img,cv.COLOR\_GRAY2BGR)

circles = [cv.HoughCircles](../../dd/d1a/group__imgproc__feature.html#ga47849c3be0d0406ad3ca45db65a25d2d "../../dd/d1a/group__imgproc__feature.html#ga47849c3be0d0406ad3ca45db65a25d2d")(img,cv.HOUGH\_GRADIENT,1,20,
 param1=50,param2=30,minRadius=0,maxRadius=0)

circles = np.uint16(np.around(circles))
for i in circles[0,:]:
 # draw the outer circle
 [cv.circle](../../d6/d6e/group__imgproc__draw.html#gaf10604b069374903dbd0f0488cb43670 "../../d6/d6e/group__imgproc__draw.html#gaf10604b069374903dbd0f0488cb43670")(cimg,(i[0],i[1]),i[2],(0,255,0),2)
 # draw the center of the circle
 [cv.circle](../../d6/d6e/group__imgproc__draw.html#gaf10604b069374903dbd0f0488cb43670 "../../d6/d6e/group__imgproc__draw.html#gaf10604b069374903dbd0f0488cb43670")(cimg,(i[0],i[1]),2,(0,0,255),3)

[cv.imshow](../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563 "../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563")('detected circles',cimg)
[cv.waitKey](../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7 "../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7")(0)
[cv.destroyAllWindows](../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481 "../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481")()
[cv::imshow](../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563 "../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563")void imshow(const String &winname, InputArray mat)Displays an image in the specified window.
[cv::waitKey](../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7 "../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7")int waitKey(int delay=0)Waits for a pressed key.
[cv::destroyAllWindows](../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481 "../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481")void destroyAllWindows()Destroys all of the HighGUI windows.
[cv::imread](../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8 "../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8")CV\_EXPORTS\_W Mat imread(const String &filename, int flags=IMREAD\_COLOR)Loads an image from a file.
[cv::cvtColor](../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab "../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab")void cvtColor(InputArray src, OutputArray dst, int code, int dstCn=0)Converts an image from one color space to another.
[cv::circle](../../d6/d6e/group__imgproc__draw.html#gaf10604b069374903dbd0f0488cb43670 "../../d6/d6e/group__imgproc__draw.html#gaf10604b069374903dbd0f0488cb43670")void circle(InputOutputArray img, Point center, int radius, const Scalar &color, int thickness=1, int lineType=LINE\_8, int shift=0)Draws a circle.
[cv::HoughCircles](../../dd/d1a/group__imgproc__feature.html#ga47849c3be0d0406ad3ca45db65a25d2d "../../dd/d1a/group__imgproc__feature.html#ga47849c3be0d0406ad3ca45db65a25d2d")void HoughCircles(InputArray image, OutputArray circles, int method, double dp, double minDist, double param1=100, double param2=100, int minRadius=0, int maxRadius=0)Finds circles in a grayscale image using the Hough transform.
[cv::medianBlur](../../d4/d86/group__imgproc__filter.html#ga564869aa33e58769b4469101aac458f9 "../../d4/d86/group__imgproc__filter.html#ga564869aa33e58769b4469101aac458f9")void medianBlur(InputArray src, OutputArray dst, int ksize)Blurs an image using the median filter.
 Result is shown below:

![](../../houghcircles2.jpg)

image
# Additional Resources

# Exercises

