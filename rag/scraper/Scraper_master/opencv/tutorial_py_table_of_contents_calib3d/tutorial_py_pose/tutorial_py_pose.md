
# Goal

In this section,

* We will learn to exploit calib3d module to create some 3D effects in images.

# Basics

This is going to be a small section. During the last session on camera calibration, you have found the camera matrix, distortion coefficients etc. Given a pattern image, we can utilize the above information to calculate its pose, or how the object is situated in space, like how it is rotated, how it is displaced etc. For a planar object, we can assume Z=0, such that, the problem now becomes how camera is placed in space to see our pattern image. So, if we know how the object lies in the space, we can draw some 2D diagrams in it to simulate the 3D effect. Let's see how to do it.

Our problem is, we want to draw our 3D coordinate axis (X, Y, Z axes) on our chessboard's first corner. X axis in blue color, Y axis in green color and Z axis in red color. So in-effect, Z axis should feel like it is perpendicular to our chessboard plane.

First, let's load the camera matrix and distortion coefficients from the previous calibration result. 

import numpy as np
import cv2 as cv
import glob

# Load previously saved data
with np.load('B.npz') as X:
 mtx, dist, \_, \_ = [X[i] for i in ('mtx','dist','rvecs','tvecs')]
 Now let's create a function, draw which takes the corners in the chessboard (obtained using **[cv.findChessboardCorners()](../../d9/d0c/group__calib3d.html#ga93efa9b0aa890de240ca32b11253dd4a "Finds the positions of internal corners of the chessboard.")**) and **axis points** to draw a 3D axis. 

def draw(img, corners, imgpts):
 corner = tuple(corners[0].ravel())
 img = [cv.line](../../d6/d6e/group__imgproc__draw.html#ga7078a9fae8c7e7d13d24dac2520ae4a2 "../../d6/d6e/group__imgproc__draw.html#ga7078a9fae8c7e7d13d24dac2520ae4a2")(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
 img = [cv.line](../../d6/d6e/group__imgproc__draw.html#ga7078a9fae8c7e7d13d24dac2520ae4a2 "../../d6/d6e/group__imgproc__draw.html#ga7078a9fae8c7e7d13d24dac2520ae4a2")(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
 img = [cv.line](../../d6/d6e/group__imgproc__draw.html#ga7078a9fae8c7e7d13d24dac2520ae4a2 "../../d6/d6e/group__imgproc__draw.html#ga7078a9fae8c7e7d13d24dac2520ae4a2")(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
 return img
[cv::line](../../d6/d6e/group__imgproc__draw.html#ga7078a9fae8c7e7d13d24dac2520ae4a2 "../../d6/d6e/group__imgproc__draw.html#ga7078a9fae8c7e7d13d24dac2520ae4a2")void line(InputOutputArray img, Point pt1, Point pt2, const Scalar &color, int thickness=1, int lineType=LINE\_8, int shift=0)Draws a line segment connecting two points.
 Then as in previous case, we create termination criteria, object points (3D points of corners in chessboard) and axis points. Axis points are points in 3D space for drawing the axis. We draw axis of length 3 (units will be in terms of chess square size since we calibrated based on that size). So our X axis is drawn from (0,0,0) to (3,0,0), so for Y axis. For Z axis, it is drawn from (0,0,0) to (0,0,-3). Negative denotes it is drawn towards the camera. 

criteria = (cv.TERM\_CRITERIA\_EPS + cv.TERM\_CRITERIA\_MAX\_ITER, 30, 0.001)
objp = np.zeros((6\*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)
 Now, as usual, we load each image. Search for 7x6 grid. If found, we refine it with subcorner pixels. Then to calculate the rotation and translation, we use the function, **[cv.solvePnPRansac()](../../d9/d0c/group__calib3d.html#ga50620f0e26e02caa2e9adc07b5fbf24e "Finds an object pose from 3D-2D point correspondences using the RANSAC scheme.")**. Once we those transformation matrices, we use them to project our **axis points** to the image plane. In simple words, we find the points on image plane corresponding to each of (3,0,0),(0,3,0),(0,0,3) in 3D space. Once we get them, we draw lines from the first corner to each of these points using our generateImage() function. Done !!! 

for fname in glob.glob('left\*.jpg'):
 img = [cv.imread](../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8 "../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8")(fname)
 gray = [cv.cvtColor](../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab "../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab")(img,cv.COLOR\_BGR2GRAY)
 ret, corners = [cv.findChessboardCorners](../../d9/d0c/group__calib3d.html#ga93efa9b0aa890de240ca32b11253dd4a "../../d9/d0c/group__calib3d.html#ga93efa9b0aa890de240ca32b11253dd4a")(gray, (7,6),None)

 if ret == True:
 corners2 = [cv.cornerSubPix](../../dd/d1a/group__imgproc__feature.html#ga354e0d7c86d0d9da75de9b9701a9a87e "../../dd/d1a/group__imgproc__feature.html#ga354e0d7c86d0d9da75de9b9701a9a87e")(gray,corners,(11,11),(-1,-1),criteria)

 # Find the rotation and translation vectors.
 ret,rvecs, tvecs = [cv.solvePnP](../../d9/d0c/group__calib3d.html#ga549c2075fac14829ff4a58bc931c033d "../../d9/d0c/group__calib3d.html#ga549c2075fac14829ff4a58bc931c033d")(objp, corners2, mtx, dist)

 # project 3D points to image plane
 imgpts, jac = [cv.projectPoints](../../d9/d0c/group__calib3d.html#ga1019495a2c8d1743ed5cc23fa0daff8c "../../d9/d0c/group__calib3d.html#ga1019495a2c8d1743ed5cc23fa0daff8c")(axis, rvecs, tvecs, mtx, dist)

 img = draw(img,corners2,imgpts)
 [cv.imshow](../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563 "../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563")('img',img)
 k = [cv.waitKey](../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7 "../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7")(0) & 0xFF
 if k == ord('s'):
 [cv.imwrite](../../d4/da8/group__imgcodecs.html#ga8ac397bd09e48851665edbe12aa28f25 "../../d4/da8/group__imgcodecs.html#ga8ac397bd09e48851665edbe12aa28f25")(fname[:6]+'.png', img)

[cv.destroyAllWindows](../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481 "../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481")()
[cv::projectPoints](../../d9/d0c/group__calib3d.html#ga1019495a2c8d1743ed5cc23fa0daff8c "../../d9/d0c/group__calib3d.html#ga1019495a2c8d1743ed5cc23fa0daff8c")void projectPoints(InputArray objectPoints, InputArray rvec, InputArray tvec, InputArray cameraMatrix, InputArray distCoeffs, OutputArray imagePoints, OutputArray jacobian=noArray(), double aspectRatio=0)Projects 3D points to an image plane.
[cv::solvePnP](../../d9/d0c/group__calib3d.html#ga549c2075fac14829ff4a58bc931c033d "../../d9/d0c/group__calib3d.html#ga549c2075fac14829ff4a58bc931c033d")bool solvePnP(InputArray objectPoints, InputArray imagePoints, InputArray cameraMatrix, InputArray distCoeffs, OutputArray rvec, OutputArray tvec, bool useExtrinsicGuess=false, int flags=SOLVEPNP\_ITERATIVE)Finds an object pose from 3D-2D point correspondences.
[cv::findChessboardCorners](../../d9/d0c/group__calib3d.html#ga93efa9b0aa890de240ca32b11253dd4a "../../d9/d0c/group__calib3d.html#ga93efa9b0aa890de240ca32b11253dd4a")bool findChessboardCorners(InputArray image, Size patternSize, OutputArray corners, int flags=CALIB\_CB\_ADAPTIVE\_THRESH+CALIB\_CB\_NORMALIZE\_IMAGE)Finds the positions of internal corners of the chessboard.
[cv::imshow](../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563 "../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563")void imshow(const String &winname, InputArray mat)Displays an image in the specified window.
[cv::waitKey](../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7 "../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7")int waitKey(int delay=0)Waits for a pressed key.
[cv::destroyAllWindows](../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481 "../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481")void destroyAllWindows()Destroys all of the HighGUI windows.
[cv::imwrite](../../d4/da8/group__imgcodecs.html#ga8ac397bd09e48851665edbe12aa28f25 "../../d4/da8/group__imgcodecs.html#ga8ac397bd09e48851665edbe12aa28f25")CV\_EXPORTS\_W bool imwrite(const String &filename, InputArray img, const std::vector< int > &params=std::vector< int >())Saves an image to a specified file.
[cv::imread](../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8 "../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8")CV\_EXPORTS\_W Mat imread(const String &filename, int flags=IMREAD\_COLOR)Loads an image from a file.
[cv::cvtColor](../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab "../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab")void cvtColor(InputArray src, OutputArray dst, int code, int dstCn=0)Converts an image from one color space to another.
[cv::cornerSubPix](../../dd/d1a/group__imgproc__feature.html#ga354e0d7c86d0d9da75de9b9701a9a87e "../../dd/d1a/group__imgproc__feature.html#ga354e0d7c86d0d9da75de9b9701a9a87e")void cornerSubPix(InputArray image, InputOutputArray corners, Size winSize, Size zeroZone, TermCriteria criteria)Refines the corner locations.
 See some results below. Notice that each axis is 3 squares long.:

![](../../pose_1.jpg)

image
## Render a Cube

If you want to draw a cube, modify the generateImage() function and axis points as follows.

Modified generateImage() function: 

def draw(img, corners, imgpts):
 imgpts = np.int32(imgpts).reshape(-1,2)

 # draw ground floor in green
 img = [cv.drawContours](../../d6/d6e/group__imgproc__draw.html#ga746c0625f1781f1ffc9056259103edbc "../../d6/d6e/group__imgproc__draw.html#ga746c0625f1781f1ffc9056259103edbc")(img, [imgpts[:4]],-1,(0,255,0),-3)

 # draw pillars in blue color
 for i,j in zip(range(4),range(4,8)):
 img = [cv.line](../../d6/d6e/group__imgproc__draw.html#ga7078a9fae8c7e7d13d24dac2520ae4a2 "../../d6/d6e/group__imgproc__draw.html#ga7078a9fae8c7e7d13d24dac2520ae4a2")(img, tuple(imgpts[i]), tuple(imgpts[j]),(255),3)

 # draw top layer in red color
 img = [cv.drawContours](../../d6/d6e/group__imgproc__draw.html#ga746c0625f1781f1ffc9056259103edbc "../../d6/d6e/group__imgproc__draw.html#ga746c0625f1781f1ffc9056259103edbc")(img, [imgpts[4:]],-1,(0,0,255),3)

 return img
[cv::drawContours](../../d6/d6e/group__imgproc__draw.html#ga746c0625f1781f1ffc9056259103edbc "../../d6/d6e/group__imgproc__draw.html#ga746c0625f1781f1ffc9056259103edbc")void drawContours(InputOutputArray image, InputArrayOfArrays contours, int contourIdx, const Scalar &color, int thickness=1, int lineType=LINE\_8, InputArray hierarchy=noArray(), int maxLevel=INT\_MAX, Point offset=Point())Draws contours outlines or filled contours.
 Modified axis points. They are the 8 corners of a cube in 3D space: 

axis = np.float32([[0,0,0], [0,3,0], [3,3,0], [3,0,0],
 [0,0,-3],[0,3,-3],[3,3,-3],[3,0,-3] ])
 And look at the result below:

![](../../pose_2.jpg)

image
If you are interested in graphics, augmented reality etc, you can use OpenGL to render more complicated figures.

# Additional Resources

# Exercises

