
**Prev Tutorial:** [Contour Properties](../../d1/d32/tutorial_py_contour_properties.html "../../d1/d32/tutorial_py_contour_properties.html")   

**Next Tutorial:** [Contours Hierarchy](../../d9/d8b/tutorial_py_contours_hierarchy.html "../../d9/d8b/tutorial_py_contours_hierarchy.html")   

# Goal

In this chapter, we will learn about

* Convexity defects and how to find them.
* Finding shortest distance from a point to a polygon
* Matching different shapes

# Theory and Code

## 1. Convexity Defects

We saw what is convex hull in second chapter about contours. Any deviation of the object from this hull can be considered as convexity defect.

OpenCV comes with a ready-made function to find this, **[cv.convexityDefects()](../../d3/dc0/group__imgproc__shape.html#gada4437098113fd8683c932e0567f47ba "Finds the convexity defects of a contour.")**. A basic function call would look like below: 

hull = [cv.convexHull](../../d3/dc0/group__imgproc__shape.html#ga014b28e56cb8854c0de4a211cb2be656 "../../d3/dc0/group__imgproc__shape.html#ga014b28e56cb8854c0de4a211cb2be656")(cnt,returnPoints = False)
defects = [cv.convexityDefects](../../d3/dc0/group__imgproc__shape.html#gada4437098113fd8683c932e0567f47ba "../../d3/dc0/group__imgproc__shape.html#gada4437098113fd8683c932e0567f47ba")(cnt,hull)
[cv::convexHull](../../d3/dc0/group__imgproc__shape.html#ga014b28e56cb8854c0de4a211cb2be656 "../../d3/dc0/group__imgproc__shape.html#ga014b28e56cb8854c0de4a211cb2be656")void convexHull(InputArray points, OutputArray hull, bool clockwise=false, bool returnPoints=true)Finds the convex hull of a point set.
[cv::convexityDefects](../../d3/dc0/group__imgproc__shape.html#gada4437098113fd8683c932e0567f47ba "../../d3/dc0/group__imgproc__shape.html#gada4437098113fd8683c932e0567f47ba")void convexityDefects(InputArray contour, InputArray convexhull, OutputArray convexityDefects)Finds the convexity defects of a contour.
NoteRemember we have to pass returnPoints = False while finding convex hull, in order to find convexity defects.
It returns an array where each row contains these values - **[ start point, end point, farthest point, approximate distance to farthest point ]**. We can visualize it using an image. We draw a line joining start point and end point, then draw a circle at the farthest point. Remember first three values returned are indices of cnt. So we have to bring those values from cnt.

import cv2 as cv
import numpy as np

img = [cv.imread](../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8 "../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8")('star.jpg')
assert img is not None, "file could not be read, check with os.path.exists()"
img\_gray = [cv.cvtColor](../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab "../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab")(img,cv.COLOR\_BGR2GRAY)
ret,thresh = [cv.threshold](../../d7/d1b/group__imgproc__misc.html#gae8a4a146d1ca78c626a53577199e9c57 "../../d7/d1b/group__imgproc__misc.html#gae8a4a146d1ca78c626a53577199e9c57")(img\_gray, 127, 255,0)
contours,hierarchy = [cv.findContours](../../d3/dc0/group__imgproc__shape.html#gadf1ad6a0b82947fa1fe3c3d497f260e0 "../../d3/dc0/group__imgproc__shape.html#gadf1ad6a0b82947fa1fe3c3d497f260e0")(thresh,2,1)
cnt = contours[0]

hull = [cv.convexHull](../../d3/dc0/group__imgproc__shape.html#ga014b28e56cb8854c0de4a211cb2be656 "../../d3/dc0/group__imgproc__shape.html#ga014b28e56cb8854c0de4a211cb2be656")(cnt,returnPoints = False)
defects = [cv.convexityDefects](../../d3/dc0/group__imgproc__shape.html#gada4437098113fd8683c932e0567f47ba "../../d3/dc0/group__imgproc__shape.html#gada4437098113fd8683c932e0567f47ba")(cnt,hull)

for i in range(defects.shape[0]):
 s,e,f,d = defects[i,0]
 start = tuple(cnt[s][0])
 end = tuple(cnt[e][0])
 far = tuple(cnt[f][0])
 [cv.line](../../d6/d6e/group__imgproc__draw.html#ga7078a9fae8c7e7d13d24dac2520ae4a2 "../../d6/d6e/group__imgproc__draw.html#ga7078a9fae8c7e7d13d24dac2520ae4a2")(img,start,end,[0,255,0],2)
 [cv.circle](../../d6/d6e/group__imgproc__draw.html#gaf10604b069374903dbd0f0488cb43670 "../../d6/d6e/group__imgproc__draw.html#gaf10604b069374903dbd0f0488cb43670")(img,far,5,[0,0,255],-1)

[cv.imshow](../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563 "../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563")('img',img)
[cv.waitKey](../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7 "../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7")(0)
[cv.destroyAllWindows](../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481 "../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481")()
[cv::imshow](../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563 "../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563")void imshow(const String &winname, InputArray mat)Displays an image in the specified window.
[cv::waitKey](../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7 "../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7")int waitKey(int delay=0)Waits for a pressed key.
[cv::destroyAllWindows](../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481 "../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481")void destroyAllWindows()Destroys all of the HighGUI windows.
[cv::imread](../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8 "../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8")CV\_EXPORTS\_W Mat imread(const String &filename, int flags=IMREAD\_COLOR)Loads an image from a file.
[cv::cvtColor](../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab "../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab")void cvtColor(InputArray src, OutputArray dst, int code, int dstCn=0)Converts an image from one color space to another.
[cv::line](../../d6/d6e/group__imgproc__draw.html#ga7078a9fae8c7e7d13d24dac2520ae4a2 "../../d6/d6e/group__imgproc__draw.html#ga7078a9fae8c7e7d13d24dac2520ae4a2")void line(InputOutputArray img, Point pt1, Point pt2, const Scalar &color, int thickness=1, int lineType=LINE\_8, int shift=0)Draws a line segment connecting two points.
[cv::circle](../../d6/d6e/group__imgproc__draw.html#gaf10604b069374903dbd0f0488cb43670 "../../d6/d6e/group__imgproc__draw.html#gaf10604b069374903dbd0f0488cb43670")void circle(InputOutputArray img, Point center, int radius, const Scalar &color, int thickness=1, int lineType=LINE\_8, int shift=0)Draws a circle.
[cv::threshold](../../d7/d1b/group__imgproc__misc.html#gae8a4a146d1ca78c626a53577199e9c57 "../../d7/d1b/group__imgproc__misc.html#gae8a4a146d1ca78c626a53577199e9c57")double threshold(InputArray src, OutputArray dst, double thresh, double maxval, int type)Applies a fixed-level threshold to each array element.
[cv::findContours](../../d3/dc0/group__imgproc__shape.html#gadf1ad6a0b82947fa1fe3c3d497f260e0 "../../d3/dc0/group__imgproc__shape.html#gadf1ad6a0b82947fa1fe3c3d497f260e0")void findContours(InputArray image, OutputArrayOfArrays contours, OutputArray hierarchy, int mode, int method, Point offset=Point())Finds contours in a binary image.
 And see the result:

![](../../defects.jpg)

image
## 2. Point Polygon Test

This function finds the shortest distance between a point in the image and a contour. It returns the distance which is negative when point is outside the contour, positive when point is inside and zero if point is on the contour.

For example, we can check the point (50,50) as follows: 

dist = [cv.pointPolygonTest](../../d3/dc0/group__imgproc__shape.html#ga1a539e8db2135af2566103705d7a5722 "../../d3/dc0/group__imgproc__shape.html#ga1a539e8db2135af2566103705d7a5722")(cnt,(50,50),True)
[cv::pointPolygonTest](../../d3/dc0/group__imgproc__shape.html#ga1a539e8db2135af2566103705d7a5722 "../../d3/dc0/group__imgproc__shape.html#ga1a539e8db2135af2566103705d7a5722")double pointPolygonTest(InputArray contour, Point2f pt, bool measureDist)Performs a point-in-contour test.
 In the function, third argument is measureDist. If it is True, it finds the signed distance. If False, it finds whether the point is inside or outside or on the contour (it returns +1, -1, 0 respectively).

NoteIf you don't want to find the distance, make sure third argument is False, because, it is a time consuming process. So, making it False gives about 2-3X speedup.
## 3. Match Shapes

OpenCV comes with a function **[cv.matchShapes()](../../d3/dc0/group__imgproc__shape.html#gaadc90cb16e2362c9bd6e7363e6e4c317 "Compares two shapes.")** which enables us to compare two shapes, or two contours and returns a metric showing the similarity. The lower the result, the better match it is. It is calculated based on the hu-moment values. Different measurement methods are explained in the docs. 

import cv2 as cv
import numpy as np

img1 = [cv.imread](../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8 "../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8")('star.jpg', cv.IMREAD\_GRAYSCALE)
img2 = [cv.imread](../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8 "../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8")('star2.jpg', cv.IMREAD\_GRAYSCALE)
assert img1 is not None, "file could not be read, check with os.path.exists()"
assert img2 is not None, "file could not be read, check with os.path.exists()"

ret, thresh = [cv.threshold](../../d7/d1b/group__imgproc__misc.html#gae8a4a146d1ca78c626a53577199e9c57 "../../d7/d1b/group__imgproc__misc.html#gae8a4a146d1ca78c626a53577199e9c57")(img1, 127, 255,0)
ret, thresh2 = [cv.threshold](../../d7/d1b/group__imgproc__misc.html#gae8a4a146d1ca78c626a53577199e9c57 "../../d7/d1b/group__imgproc__misc.html#gae8a4a146d1ca78c626a53577199e9c57")(img2, 127, 255,0)
contours,hierarchy = [cv.findContours](../../d3/dc0/group__imgproc__shape.html#gadf1ad6a0b82947fa1fe3c3d497f260e0 "../../d3/dc0/group__imgproc__shape.html#gadf1ad6a0b82947fa1fe3c3d497f260e0")(thresh,2,1)
cnt1 = contours[0]
contours,hierarchy = [cv.findContours](../../d3/dc0/group__imgproc__shape.html#gadf1ad6a0b82947fa1fe3c3d497f260e0 "../../d3/dc0/group__imgproc__shape.html#gadf1ad6a0b82947fa1fe3c3d497f260e0")(thresh2,2,1)
cnt2 = contours[0]

ret = [cv.matchShapes](../../d3/dc0/group__imgproc__shape.html#gaadc90cb16e2362c9bd6e7363e6e4c317 "../../d3/dc0/group__imgproc__shape.html#gaadc90cb16e2362c9bd6e7363e6e4c317")(cnt1,cnt2,1,0.0)
print( ret )
[cv::matchShapes](../../d3/dc0/group__imgproc__shape.html#gaadc90cb16e2362c9bd6e7363e6e4c317 "../../d3/dc0/group__imgproc__shape.html#gaadc90cb16e2362c9bd6e7363e6e4c317")double matchShapes(InputArray contour1, InputArray contour2, int method, double parameter)Compares two shapes.
 I tried matching shapes with different shapes given below:

![](../../matchshapes.jpg)

image
I got following results:

* Matching Image A with itself = 0.0
* Matching Image A with Image B = 0.001946
* Matching Image A with Image C = 0.326911

See, even image rotation doesn't affect much on this comparison.

Note[Hu-Moments](https://en.wikipedia.org/wiki/Image_moment#Rotation_invariant_moments "https://en.wikipedia.org/wiki/Image_moment#Rotation_invariant_moments") are seven moments invariant to translation, rotation and scale. Seventh one is skew-invariant. Those values can be found using **[cv.HuMoments()](../../d3/dc0/group__imgproc__shape.html#gab001db45c1f1af6cbdbe64df04c4e944 "Calculates seven Hu invariants.")** function.
# Additional Resources

## Exercises

1. Check the documentation for **[cv.pointPolygonTest()](../../d3/dc0/group__imgproc__shape.html#ga1a539e8db2135af2566103705d7a5722 "Performs a point-in-contour test.")**, you can find a nice image in Red and Blue color. It represents the distance from all pixels to the white curve on it. All pixels inside curve is blue depending on the distance. Similarly outside points are red. Contour edges are marked with White. So problem is simple. Write a code to create such a representation of distance.
2. Compare images of digits or letters using **[cv.matchShapes()](../../d3/dc0/group__imgproc__shape.html#gaadc90cb16e2362c9bd6e7363e6e4c317 "Compares two shapes.")**. ( That would be a simple step towards OCR )

