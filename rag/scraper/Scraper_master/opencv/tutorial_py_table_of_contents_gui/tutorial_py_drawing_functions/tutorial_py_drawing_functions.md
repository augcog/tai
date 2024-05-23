
# Goal

* Learn to draw different geometric shapes with OpenCV
* You will learn these functions : **[cv.line()](../../d6/d6e/group__imgproc__draw.html#ga7078a9fae8c7e7d13d24dac2520ae4a2 "Draws a line segment connecting two points.")**, **[cv.circle()](../../d6/d6e/group__imgproc__draw.html#gaf10604b069374903dbd0f0488cb43670 "Draws a circle.")** , **[cv.rectangle()](../../d6/d6e/group__imgproc__draw.html#ga07d2f74cadcf8e305e810ce8eed13bc9 "Draws a simple, thick, or filled up-right rectangle.")**, **[cv.ellipse()](../../d6/d6e/group__imgproc__draw.html#ga28b2267d35786f5f890ca167236cbc69 "Draws a simple or thick elliptic arc or fills an ellipse sector.")**, **[cv.putText()](../../d6/d6e/group__imgproc__draw.html#ga5126f47f883d730f633d74f07456c576 "Draws a text string.")** etc.

# Code

In all the above functions, you will see some common arguments as given below:

* img : The image where you want to draw the shapes
* color : Color of the shape. for BGR, pass it as a tuple, eg: (255,0,0) for blue. For grayscale, just pass the scalar value.
* thickness : Thickness of the line or circle etc. If **-1** is passed for closed figures like circles, it will fill the shape. *default thickness = 1*
* lineType : Type of line, whether 8-connected, anti-aliased line etc. *By default, it is 8-connected.* [cv.LINE\_AA](../../d6/d6e/group__imgproc__draw.html#ggaf076ef45de481ac96e0ab3dc2c29a777a85fdabe5335c9e6656563dfd7c94fb4f "antialiased line") gives anti-aliased line which looks great for curves.

## Drawing Line

To draw a line, you need to pass starting and ending coordinates of line. We will create a black image and draw a blue line on it from top-left to bottom-right corners. 

import numpy as np
import cv2 as cv

# Create a black image
img = np.zeros((512,512,3), np.uint8)

# Draw a diagonal blue line with thickness of 5 px
[cv.line](../../d6/d6e/group__imgproc__draw.html#ga7078a9fae8c7e7d13d24dac2520ae4a2 "../../d6/d6e/group__imgproc__draw.html#ga7078a9fae8c7e7d13d24dac2520ae4a2")(img,(0,0),(511,511),(255,0,0),5)
[cv::line](../../d6/d6e/group__imgproc__draw.html#ga7078a9fae8c7e7d13d24dac2520ae4a2 "../../d6/d6e/group__imgproc__draw.html#ga7078a9fae8c7e7d13d24dac2520ae4a2")void line(InputOutputArray img, Point pt1, Point pt2, const Scalar &color, int thickness=1, int lineType=LINE\_8, int shift=0)Draws a line segment connecting two points.
 ## Drawing Rectangle

To draw a rectangle, you need top-left corner and bottom-right corner of rectangle. This time we will draw a green rectangle at the top-right corner of image. 

[cv.rectangle](../../d6/d6e/group__imgproc__draw.html#ga07d2f74cadcf8e305e810ce8eed13bc9 "../../d6/d6e/group__imgproc__draw.html#ga07d2f74cadcf8e305e810ce8eed13bc9")(img,(384,0),(510,128),(0,255,0),3)
[cv::rectangle](../../d6/d6e/group__imgproc__draw.html#ga07d2f74cadcf8e305e810ce8eed13bc9 "../../d6/d6e/group__imgproc__draw.html#ga07d2f74cadcf8e305e810ce8eed13bc9")void rectangle(InputOutputArray img, Point pt1, Point pt2, const Scalar &color, int thickness=1, int lineType=LINE\_8, int shift=0)Draws a simple, thick, or filled up-right rectangle.
 ## Drawing Circle

To draw a circle, you need its center coordinates and radius. We will draw a circle inside the rectangle drawn above. 

[cv.circle](../../d6/d6e/group__imgproc__draw.html#gaf10604b069374903dbd0f0488cb43670 "../../d6/d6e/group__imgproc__draw.html#gaf10604b069374903dbd0f0488cb43670")(img,(447,63), 63, (0,0,255), -1)
[cv::circle](../../d6/d6e/group__imgproc__draw.html#gaf10604b069374903dbd0f0488cb43670 "../../d6/d6e/group__imgproc__draw.html#gaf10604b069374903dbd0f0488cb43670")void circle(InputOutputArray img, Point center, int radius, const Scalar &color, int thickness=1, int lineType=LINE\_8, int shift=0)Draws a circle.
 ## Drawing Ellipse

To draw the ellipse, we need to pass several arguments. One argument is the center location (x,y). Next argument is axes lengths (major axis length, minor axis length). angle is the angle of rotation of ellipse in anti-clockwise direction. startAngle and endAngle denotes the starting and ending of ellipse arc measured in clockwise direction from major axis. i.e. giving values 0 and 360 gives the full ellipse. For more details, check the documentation of **[cv.ellipse()](../../d6/d6e/group__imgproc__draw.html#ga28b2267d35786f5f890ca167236cbc69 "Draws a simple or thick elliptic arc or fills an ellipse sector.")**. Below example draws a half ellipse at the center of the image. 

[cv.ellipse](../../d6/d6e/group__imgproc__draw.html#ga28b2267d35786f5f890ca167236cbc69 "../../d6/d6e/group__imgproc__draw.html#ga28b2267d35786f5f890ca167236cbc69")(img,(256,256),(100,50),0,0,180,255,-1)
[cv::ellipse](../../d6/d6e/group__imgproc__draw.html#ga28b2267d35786f5f890ca167236cbc69 "../../d6/d6e/group__imgproc__draw.html#ga28b2267d35786f5f890ca167236cbc69")void ellipse(InputOutputArray img, Point center, Size axes, double angle, double startAngle, double endAngle, const Scalar &color, int thickness=1, int lineType=LINE\_8, int shift=0)Draws a simple or thick elliptic arc or fills an ellipse sector.
 ## Drawing Polygon

To draw a polygon, first you need coordinates of vertices. Make those points into an array of shape ROWSx1x2 where ROWS are number of vertices and it should be of type int32. Here we draw a small polygon of with four vertices in yellow color. 

pts = np.array([[10,5],[20,30],[70,20],[50,10]], np.int32)
pts = pts.reshape((-1,1,2))
[cv.polylines](../../d6/d6e/group__imgproc__draw.html#gaa3c25f9fb764b6bef791bf034f6e26f5 "../../d6/d6e/group__imgproc__draw.html#gaa3c25f9fb764b6bef791bf034f6e26f5")(img,[pts],True,(0,255,255))
[cv::polylines](../../d6/d6e/group__imgproc__draw.html#gaa3c25f9fb764b6bef791bf034f6e26f5 "../../d6/d6e/group__imgproc__draw.html#gaa3c25f9fb764b6bef791bf034f6e26f5")void polylines(InputOutputArray img, InputArrayOfArrays pts, bool isClosed, const Scalar &color, int thickness=1, int lineType=LINE\_8, int shift=0)Draws several polygonal curves.
NoteIf third argument is False, you will get a polylines joining all the points, not a closed shape.

[cv.polylines()](../../d6/d6e/group__imgproc__draw.html#gaa3c25f9fb764b6bef791bf034f6e26f5 "Draws several polygonal curves.") can be used to draw multiple lines. Just create a list of all the lines you want to draw and pass it to the function. All lines will be drawn individually. It is a much better and faster way to draw a group of lines than calling [cv.line()](../../d6/d6e/group__imgproc__draw.html#ga7078a9fae8c7e7d13d24dac2520ae4a2 "Draws a line segment connecting two points.") for each line.
## Adding Text to Images:

To put texts in images, you need specify following things.

* Text data that you want to write
* Position coordinates of where you want put it (i.e. bottom-left corner where data starts).
* Font type (Check **[cv.putText()](../../d6/d6e/group__imgproc__draw.html#ga5126f47f883d730f633d74f07456c576 "Draws a text string.")** docs for supported fonts)
* Font Scale (specifies the size of font)
* regular things like color, thickness, lineType etc. For better look, lineType = [cv.LINE\_AA](../../d6/d6e/group__imgproc__draw.html#ggaf076ef45de481ac96e0ab3dc2c29a777a85fdabe5335c9e6656563dfd7c94fb4f "antialiased line") is recommended.

We will write **OpenCV** on our image in white color. 

font = cv.FONT\_HERSHEY\_SIMPLEX
[cv.putText](../../d6/d6e/group__imgproc__draw.html#ga5126f47f883d730f633d74f07456c576 "../../d6/d6e/group__imgproc__draw.html#ga5126f47f883d730f633d74f07456c576")(img,'OpenCV',(10,500), font, 4,(255,255,255),2,cv.LINE\_AA)
[cv::putText](../../d6/d6e/group__imgproc__draw.html#ga5126f47f883d730f633d74f07456c576 "../../d6/d6e/group__imgproc__draw.html#ga5126f47f883d730f633d74f07456c576")void putText(InputOutputArray img, const String &text, Point org, int fontFace, double fontScale, Scalar color, int thickness=1, int lineType=LINE\_8, bool bottomLeftOrigin=false)Draws a text string.
## Result

So it is time to see the final result of our drawing. As you studied in previous articles, display the image to see it.

![](../../drawing_result.jpg)

image
# Additional Resources

1. The angles used in ellipse function is not our circular angles. For more details, visit [this discussion](http://answers.opencv.org/question/14541/angles-in-ellipse-function/ "http://answers.opencv.org/question/14541/angles-in-ellipse-function/").

# Exercises

1. Try to create the logo of OpenCV using drawing functions available in OpenCV.

