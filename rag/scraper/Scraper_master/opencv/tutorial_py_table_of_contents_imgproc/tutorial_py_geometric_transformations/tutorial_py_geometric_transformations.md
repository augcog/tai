
# Goals

* Learn to apply different geometric transformations to images, like translation, rotation, affine transformation etc.
* You will see these functions: **[cv.getPerspectiveTransform](../../da/d54/group__imgproc__transform.html#ga20f62aa3235d869c9956436c870893ae "Calculates a perspective transform from four pairs of the corresponding points.")**

# Transformations

OpenCV provides two transformation functions, **[cv.warpAffine](../../da/d54/group__imgproc__transform.html#ga0203d9ee5fcd28d40dbc4a1ea4451983 "Applies an affine transformation to an image.")** and **[cv.warpPerspective](../../da/d54/group__imgproc__transform.html#gaf73673a7e8e18ec6963e3774e6a94b87 "Applies a perspective transformation to an image.")**, with which you can perform all kinds of transformations. **[cv.warpAffine](../../da/d54/group__imgproc__transform.html#ga0203d9ee5fcd28d40dbc4a1ea4451983 "Applies an affine transformation to an image.")** takes a 2x3 transformation matrix while **[cv.warpPerspective](../../da/d54/group__imgproc__transform.html#gaf73673a7e8e18ec6963e3774e6a94b87 "Applies a perspective transformation to an image.")** takes a 3x3 transformation matrix as input.

## Scaling

Scaling is just resizing of the image. OpenCV comes with a function **[cv.resize()](../../da/d54/group__imgproc__transform.html#ga47a974309e9102f5f08231edc7e7529d "Resizes an image.")** for this purpose. The size of the image can be specified manually, or you can specify the scaling factor. Different interpolation methods are used. Preferable interpolation methods are **[cv.INTER\_AREA](../../da/d54/group__imgproc__transform.html#gga5bb5a1fea74ea38e1a5445ca803ff121acf959dca2480cc694ca016b81b442ceb "../../da/d54/group__imgproc__transform.html#gga5bb5a1fea74ea38e1a5445ca803ff121acf959dca2480cc694ca016b81b442ceb")** for shrinking and **[cv.INTER\_CUBIC](../../da/d54/group__imgproc__transform.html#gga5bb5a1fea74ea38e1a5445ca803ff121a55e404e7fa9684af79fe9827f36a5dc1 "../../da/d54/group__imgproc__transform.html#gga5bb5a1fea74ea38e1a5445ca803ff121a55e404e7fa9684af79fe9827f36a5dc1")** (slow) & **[cv.INTER\_LINEAR](../../da/d54/group__imgproc__transform.html#gga5bb5a1fea74ea38e1a5445ca803ff121ac97d8e4880d8b5d509e96825c7522deb "../../da/d54/group__imgproc__transform.html#gga5bb5a1fea74ea38e1a5445ca803ff121ac97d8e4880d8b5d509e96825c7522deb")** for zooming. By default, the interpolation method **[cv.INTER\_LINEAR](../../da/d54/group__imgproc__transform.html#gga5bb5a1fea74ea38e1a5445ca803ff121ac97d8e4880d8b5d509e96825c7522deb "../../da/d54/group__imgproc__transform.html#gga5bb5a1fea74ea38e1a5445ca803ff121ac97d8e4880d8b5d509e96825c7522deb")** is used for all resizing purposes. You can resize an input image with either of following methods: 

import numpy as np
import cv2 as cv

img = [cv.imread](../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8 "../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8")('messi5.jpg')
assert img is not None, "file could not be read, check with os.path.exists()"

res = [cv.resize](../../da/d54/group__imgproc__transform.html#ga47a974309e9102f5f08231edc7e7529d "../../da/d54/group__imgproc__transform.html#ga47a974309e9102f5f08231edc7e7529d")(img,None,fx=2, fy=2, interpolation = cv.INTER\_CUBIC)

#OR

height, width = img.shape[:2]
res = [cv.resize](../../da/d54/group__imgproc__transform.html#ga47a974309e9102f5f08231edc7e7529d "../../da/d54/group__imgproc__transform.html#ga47a974309e9102f5f08231edc7e7529d")(img,(2\*width, 2\*height), interpolation = cv.INTER\_CUBIC)
[cv::imread](../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8 "../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8")CV\_EXPORTS\_W Mat imread(const String &filename, int flags=IMREAD\_COLOR)Loads an image from a file.
[cv::resize](../../da/d54/group__imgproc__transform.html#ga47a974309e9102f5f08231edc7e7529d "../../da/d54/group__imgproc__transform.html#ga47a974309e9102f5f08231edc7e7529d")void resize(InputArray src, OutputArray dst, Size dsize, double fx=0, double fy=0, int interpolation=INTER\_LINEAR)Resizes an image.
 ## Translation

Translation is the shifting of an object's location. If you know the shift in the (x,y) direction and let it be \((t\_x,t\_y)\), you can create the transformation matrix \(\textbf{M}\) as follows:

\[M = \begin{bmatrix} 1 & 0 & t\_x \\ 0 & 1 & t\_y \end{bmatrix}\]

You can take make it into a Numpy array of type np.float32 and pass it into the **[cv.warpAffine()](../../da/d54/group__imgproc__transform.html#ga0203d9ee5fcd28d40dbc4a1ea4451983 "Applies an affine transformation to an image.")** function. See the below example for a shift of (100,50): 

import numpy as np
import cv2 as cv

img = [cv.imread](../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8 "../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8")('messi5.jpg', cv.IMREAD\_GRAYSCALE)
assert img is not None, "file could not be read, check with os.path.exists()"
rows,cols = img.shape

M = np.float32([[1,0,100],[0,1,50]])
dst = [cv.warpAffine](../../da/d54/group__imgproc__transform.html#ga0203d9ee5fcd28d40dbc4a1ea4451983 "../../da/d54/group__imgproc__transform.html#ga0203d9ee5fcd28d40dbc4a1ea4451983")(img,M,(cols,rows))

[cv.imshow](../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563 "../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563")('img',dst)
[cv.waitKey](../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7 "../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7")(0)
[cv.destroyAllWindows](../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481 "../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481")()
[cv::imshow](../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563 "../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563")void imshow(const String &winname, InputArray mat)Displays an image in the specified window.
[cv::waitKey](../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7 "../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7")int waitKey(int delay=0)Waits for a pressed key.
[cv::destroyAllWindows](../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481 "../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481")void destroyAllWindows()Destroys all of the HighGUI windows.
[cv::warpAffine](../../da/d54/group__imgproc__transform.html#ga0203d9ee5fcd28d40dbc4a1ea4451983 "../../da/d54/group__imgproc__transform.html#ga0203d9ee5fcd28d40dbc4a1ea4451983")void warpAffine(InputArray src, OutputArray dst, InputArray M, Size dsize, int flags=INTER\_LINEAR, int borderMode=BORDER\_CONSTANT, const Scalar &borderValue=Scalar())Applies an affine transformation to an image.
 **warning**

The third argument of the **[cv.warpAffine()](../../da/d54/group__imgproc__transform.html#ga0203d9ee5fcd28d40dbc4a1ea4451983 "Applies an affine transformation to an image.")** function is the size of the output image, which should be in the form of \*\*(width, height)\*\*. Remember width = number of columns, and height = number of rows.

See the result below:

![](../../translation.jpg)

image
## Rotation

Rotation of an image for an angle \(\theta\) is achieved by the transformation matrix of the form

\[M = \begin{bmatrix} cos\theta & -sin\theta \\ sin\theta & cos\theta \end{bmatrix}\]

But OpenCV provides scaled rotation with adjustable center of rotation so that you can rotate at any location you prefer. The modified transformation matrix is given by

\[\begin{bmatrix} \alpha & \beta & (1- \alpha ) \cdot center.x - \beta \cdot center.y \\ - \beta & \alpha & \beta \cdot center.x + (1- \alpha ) \cdot center.y \end{bmatrix}\]

where:

\[\begin{array}{l} \alpha = scale \cdot \cos \theta , \\ \beta = scale \cdot \sin \theta \end{array}\]

To find this transformation matrix, OpenCV provides a function, **[cv.getRotationMatrix2D](../../da/d54/group__imgproc__transform.html#gafbbc470ce83812914a70abfb604f4326 "Calculates an affine matrix of 2D rotation.")**. Check out the below example which rotates the image by 90 degree with respect to center without any scaling. 

img = [cv.imread](../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8 "../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8")('messi5.jpg', cv.IMREAD\_GRAYSCALE)
assert img is not None, "file could not be read, check with os.path.exists()"
rows,cols = img.shape

# cols-1 and rows-1 are the coordinate limits.
M = [cv.getRotationMatrix2D](../../da/d54/group__imgproc__transform.html#gafbbc470ce83812914a70abfb604f4326 "../../da/d54/group__imgproc__transform.html#gafbbc470ce83812914a70abfb604f4326")(((cols-1)/2.0,(rows-1)/2.0),90,1)
dst = [cv.warpAffine](../../da/d54/group__imgproc__transform.html#ga0203d9ee5fcd28d40dbc4a1ea4451983 "../../da/d54/group__imgproc__transform.html#ga0203d9ee5fcd28d40dbc4a1ea4451983")(img,M,(cols,rows))
[cv::getRotationMatrix2D](../../da/d54/group__imgproc__transform.html#gafbbc470ce83812914a70abfb604f4326 "../../da/d54/group__imgproc__transform.html#gafbbc470ce83812914a70abfb604f4326")Mat getRotationMatrix2D(Point2f center, double angle, double scale)Calculates an affine matrix of 2D rotation.**Definition** imgproc.hpp:2585
 See the result:

![](../../rotation.jpg)

image
## Affine Transformation

In affine transformation, all parallel lines in the original image will still be parallel in the output image. To find the transformation matrix, we need three points from the input image and their corresponding locations in the output image. Then **[cv.getAffineTransform](../../da/d54/group__imgproc__transform.html#ga8f6d378f9f8eebb5cb55cd3ae295a999 "Calculates an affine transform from three pairs of the corresponding points.")** will create a 2x3 matrix which is to be passed to **[cv.warpAffine](../../da/d54/group__imgproc__transform.html#ga0203d9ee5fcd28d40dbc4a1ea4451983 "Applies an affine transformation to an image.")**.

Check the below example, and also look at the points I selected (which are marked in green color): 

img = [cv.imread](../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8 "../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8")('drawing.png')
assert img is not None, "file could not be read, check with os.path.exists()"
rows,cols,ch = img.shape

pts1 = np.float32([[50,50],[200,50],[50,200]])
pts2 = np.float32([[10,100],[200,50],[100,250]])

M = [cv.getAffineTransform](../../da/d54/group__imgproc__transform.html#ga8f6d378f9f8eebb5cb55cd3ae295a999 "../../da/d54/group__imgproc__transform.html#ga8f6d378f9f8eebb5cb55cd3ae295a999")(pts1,pts2)

dst = [cv.warpAffine](../../da/d54/group__imgproc__transform.html#ga0203d9ee5fcd28d40dbc4a1ea4451983 "../../da/d54/group__imgproc__transform.html#ga0203d9ee5fcd28d40dbc4a1ea4451983")(img,M,(cols,rows))

plt.subplot(121),plt.imshow(img),plt.title('Input')
plt.subplot(122),plt.imshow(dst),plt.title('Output')
plt.show()
[cv::getAffineTransform](../../da/d54/group__imgproc__transform.html#ga8f6d378f9f8eebb5cb55cd3ae295a999 "../../da/d54/group__imgproc__transform.html#ga8f6d378f9f8eebb5cb55cd3ae295a999")Mat getAffineTransform(const Point2f src[], const Point2f dst[])Calculates an affine transform from three pairs of the corresponding points.
 See the result:

![](../../affine.jpg)

image
## Perspective Transformation

For perspective transformation, you need a 3x3 transformation matrix. Straight lines will remain straight even after the transformation. To find this transformation matrix, you need 4 points on the input image and corresponding points on the output image. Among these 4 points, 3 of them should not be collinear. Then the transformation matrix can be found by the function **[cv.getPerspectiveTransform](../../da/d54/group__imgproc__transform.html#ga20f62aa3235d869c9956436c870893ae "Calculates a perspective transform from four pairs of the corresponding points.")**. Then apply **[cv.warpPerspective](../../da/d54/group__imgproc__transform.html#gaf73673a7e8e18ec6963e3774e6a94b87 "Applies a perspective transformation to an image.")** with this 3x3 transformation matrix.

See the code below: 

img = [cv.imread](../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8 "../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8")('sudoku.png')
assert img is not None, "file could not be read, check with os.path.exists()"
rows,cols,ch = img.shape

pts1 = np.float32([[56,65],[368,52],[28,387],[389,390]])
pts2 = np.float32([[0,0],[300,0],[0,300],[300,300]])

M = [cv.getPerspectiveTransform](../../da/d54/group__imgproc__transform.html#ga20f62aa3235d869c9956436c870893ae "../../da/d54/group__imgproc__transform.html#ga20f62aa3235d869c9956436c870893ae")(pts1,pts2)

dst = [cv.warpPerspective](../../da/d54/group__imgproc__transform.html#gaf73673a7e8e18ec6963e3774e6a94b87 "../../da/d54/group__imgproc__transform.html#gaf73673a7e8e18ec6963e3774e6a94b87")(img,M,(300,300))

plt.subplot(121),plt.imshow(img),plt.title('Input')
plt.subplot(122),plt.imshow(dst),plt.title('Output')
plt.show()
[cv::getPerspectiveTransform](../../da/d54/group__imgproc__transform.html#ga20f62aa3235d869c9956436c870893ae "../../da/d54/group__imgproc__transform.html#ga20f62aa3235d869c9956436c870893ae")Mat getPerspectiveTransform(InputArray src, InputArray dst, int solveMethod=DECOMP\_LU)Calculates a perspective transform from four pairs of the corresponding points.
[cv::warpPerspective](../../da/d54/group__imgproc__transform.html#gaf73673a7e8e18ec6963e3774e6a94b87 "../../da/d54/group__imgproc__transform.html#gaf73673a7e8e18ec6963e3774e6a94b87")void warpPerspective(InputArray src, OutputArray dst, InputArray M, Size dsize, int flags=INTER\_LINEAR, int borderMode=BORDER\_CONSTANT, const Scalar &borderValue=Scalar())Applies a perspective transformation to an image.
 Result:

![](../../perspective.jpg)

image
# Additional Resources

1. "Computer Vision: Algorithms and Applications", Richard Szeliski

# Exercises

