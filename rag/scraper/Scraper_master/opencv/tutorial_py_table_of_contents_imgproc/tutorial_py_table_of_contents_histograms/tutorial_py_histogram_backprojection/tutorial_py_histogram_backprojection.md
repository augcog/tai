
# Goal

In this chapter, we will learn about histogram backprojection.

# Theory

It was proposed by **Michael J. Swain , Dana H. Ballard** in their paper **Indexing via color histograms**.

**What is it actually in simple words?** It is used for image segmentation or finding objects of interest in an image. In simple words, it creates an image of the same size (but single channel) as that of our input image, where each pixel corresponds to the probability of that pixel belonging to our object. In more simpler words, the output image will have our object of interest in more white compared to remaining part. Well, that is an intuitive explanation. (I can't make it more simpler). Histogram Backprojection is used with camshift algorithm etc.

**How do we do it ?** We create a histogram of an image containing our object of interest (in our case, the ground, leaving player and other things). The object should fill the image as far as possible for better results. And a color histogram is preferred over grayscale histogram, because color of the object is a better way to define the object than its grayscale intensity. We then "back-project" this histogram over our test image where we need to find the object, ie in other words, we calculate the probability of every pixel belonging to the ground and show it. The resulting output on proper thresholding gives us the ground alone.

# Algorithm in Numpy

1. First we need to calculate the color histogram of both the object we need to find (let it be 'M') and the image where we are going to search (let it be 'I'). import numpy as np
import cv2 as cvfrom matplotlib import pyplot as plt

#roi is the object or region of object we need to find
roi = [cv.imread](../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8 "../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8")('rose\_red.png')
assert roi is not None, "file could not be read, check with os.path.exists()"
hsv = [cv.cvtColor](../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab "../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab")(roi,cv.COLOR\_BGR2HSV)

#target is the image we search in
target = [cv.imread](../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8 "../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8")('rose.png')
assert target is not None, "file could not be read, check with os.path.exists()"
hsvt = [cv.cvtColor](../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab "../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab")(target,cv.COLOR\_BGR2HSV)

# Find the histograms using calcHist. Can be done with np.histogram2d also
M = [cv.calcHist](../../d6/dc7/group__imgproc__hist.html#ga4b2b5fd75503ff9e6844cc4dcdaed35d "../../d6/dc7/group__imgproc__hist.html#ga4b2b5fd75503ff9e6844cc4dcdaed35d")([hsv],[0, 1], None, [180, 256], [0, 180, 0, 256] )
I = [cv.calcHist](../../d6/dc7/group__imgproc__hist.html#ga4b2b5fd75503ff9e6844cc4dcdaed35d "../../d6/dc7/group__imgproc__hist.html#ga4b2b5fd75503ff9e6844cc4dcdaed35d")([hsvt],[0, 1], None, [180, 256], [0, 180, 0, 256] )
[cv::imread](../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8 "../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8")CV\_EXPORTS\_W Mat imread(const String &filename, int flags=IMREAD\_COLOR)Loads an image from a file.
[cv::cvtColor](../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab "../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab")void cvtColor(InputArray src, OutputArray dst, int code, int dstCn=0)Converts an image from one color space to another.
[cv::calcHist](../../d6/dc7/group__imgproc__hist.html#ga4b2b5fd75503ff9e6844cc4dcdaed35d "../../d6/dc7/group__imgproc__hist.html#ga4b2b5fd75503ff9e6844cc4dcdaed35d")void calcHist(const Mat \*images, int nimages, const int \*channels, InputArray mask, OutputArray hist, int dims, const int \*histSize, const float \*\*ranges, bool uniform=true, bool accumulate=false)Calculates a histogram of a set of arrays.
2. Find the ratio \(R = \frac{M}{I}\). Then backproject R, ie use R as palette and create a new image with every pixel as its corresponding probability of being target. ie B(x,y) = R[h(x,y),s(x,y)] where h is hue and s is saturation of the pixel at (x,y). After that apply the condition \(B(x,y) = min[B(x,y), 1]\). h,s,v = [cv.split](../../d2/de8/group__core__array.html#ga0547c7fed86152d7e9d0096029c8518a "../../d2/de8/group__core__array.html#ga0547c7fed86152d7e9d0096029c8518a")(hsvt)
B = R[h.ravel(),s.ravel()]
B = np.minimum(B,1)
B = B.reshape(hsvt.shape[:2])
[cv::split](../../d2/de8/group__core__array.html#ga0547c7fed86152d7e9d0096029c8518a "../../d2/de8/group__core__array.html#ga0547c7fed86152d7e9d0096029c8518a")void split(const Mat &src, Mat \*mvbegin)Divides a multi-channel array into several single-channel arrays.
3. Now apply a convolution with a circular disc, \(B = D \ast B\), where D is the disc kernel. disc = [cv.getStructuringElement](../../d4/d86/group__imgproc__filter.html#gac342a1bb6eabf6f55c803b09268e36dc "../../d4/d86/group__imgproc__filter.html#gac342a1bb6eabf6f55c803b09268e36dc")(cv.MORPH\_ELLIPSE,(5,5))
[cv.filter2D](../../d4/d86/group__imgproc__filter.html#ga27c049795ce870216ddfb366086b5a04 "../../d4/d86/group__imgproc__filter.html#ga27c049795ce870216ddfb366086b5a04")(B,-1,disc,B)
B = np.uint8(B)
[cv.normalize](../../d2/de8/group__core__array.html#ga87eef7ee3970f86906d69a92cbf064bd "../../d2/de8/group__core__array.html#ga87eef7ee3970f86906d69a92cbf064bd")(B,B,0,255,cv.NORM\_MINMAX)
[cv::normalize](../../d2/de8/group__core__array.html#ga87eef7ee3970f86906d69a92cbf064bd "../../d2/de8/group__core__array.html#ga87eef7ee3970f86906d69a92cbf064bd")void normalize(InputArray src, InputOutputArray dst, double alpha=1, double beta=0, int norm\_type=NORM\_L2, int dtype=-1, InputArray mask=noArray())Normalizes the norm or value range of an array.
[cv::filter2D](../../d4/d86/group__imgproc__filter.html#ga27c049795ce870216ddfb366086b5a04 "../../d4/d86/group__imgproc__filter.html#ga27c049795ce870216ddfb366086b5a04")void filter2D(InputArray src, OutputArray dst, int ddepth, InputArray kernel, Point anchor=Point(-1,-1), double delta=0, int borderType=BORDER\_DEFAULT)Convolves an image with the kernel.
[cv::getStructuringElement](../../d4/d86/group__imgproc__filter.html#gac342a1bb6eabf6f55c803b09268e36dc "../../d4/d86/group__imgproc__filter.html#gac342a1bb6eabf6f55c803b09268e36dc")Mat getStructuringElement(int shape, Size ksize, Point anchor=Point(-1,-1))Returns a structuring element of the specified size and shape for morphological operations.
4. Now the location of maximum intensity gives us the location of object. If we are expecting a region in the image, thresholding for a suitable value gives a nice result. ret,thresh = [cv.threshold](../../d7/d1b/group__imgproc__misc.html#gae8a4a146d1ca78c626a53577199e9c57 "../../d7/d1b/group__imgproc__misc.html#gae8a4a146d1ca78c626a53577199e9c57")(B,50,255,0)
[cv::threshold](../../d7/d1b/group__imgproc__misc.html#gae8a4a146d1ca78c626a53577199e9c57 "../../d7/d1b/group__imgproc__misc.html#gae8a4a146d1ca78c626a53577199e9c57")double threshold(InputArray src, OutputArray dst, double thresh, double maxval, int type)Applies a fixed-level threshold to each array element.
 That's it !!

# Backprojection in OpenCV

OpenCV provides an inbuilt function **[cv.calcBackProject()](../../d6/dc7/group__imgproc__hist.html#ga3a0af640716b456c3d14af8aee12e3ca "Calculates the back projection of a histogram.")**. Its parameters are almost same as the **[cv.calcHist()](../../d6/dc7/group__imgproc__hist.html#ga4b2b5fd75503ff9e6844cc4dcdaed35d "Calculates a histogram of a set of arrays.")** function. One of its parameter is histogram which is histogram of the object and we have to find it. Also, the object histogram should be normalized before passing on to the backproject function. It returns the probability image. Then we convolve the image with a disc kernel and apply threshold. Below is my code and output : 

import numpy as np
import cv2 as cv

roi = [cv.imread](../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8 "../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8")('rose\_red.png')
assert roi is not None, "file could not be read, check with os.path.exists()"
hsv = [cv.cvtColor](../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab "../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab")(roi,cv.COLOR\_BGR2HSV)

target = [cv.imread](../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8 "../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8")('rose.png')
assert target is not None, "file could not be read, check with os.path.exists()"
hsvt = [cv.cvtColor](../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab "../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab")(target,cv.COLOR\_BGR2HSV)

# calculating object histogram
roihist = [cv.calcHist](../../d6/dc7/group__imgproc__hist.html#ga4b2b5fd75503ff9e6844cc4dcdaed35d "../../d6/dc7/group__imgproc__hist.html#ga4b2b5fd75503ff9e6844cc4dcdaed35d")([hsv],[0, 1], None, [180, 256], [0, 180, 0, 256] )

# normalize histogram and apply backprojection
[cv.normalize](../../d2/de8/group__core__array.html#ga87eef7ee3970f86906d69a92cbf064bd "../../d2/de8/group__core__array.html#ga87eef7ee3970f86906d69a92cbf064bd")(roihist,roihist,0,255,cv.NORM\_MINMAX)
dst = [cv.calcBackProject](../../d6/dc7/group__imgproc__hist.html#ga3a0af640716b456c3d14af8aee12e3ca "../../d6/dc7/group__imgproc__hist.html#ga3a0af640716b456c3d14af8aee12e3ca")([hsvt],[0,1],roihist,[0,180,0,256],1)

# Now convolute with circular disc
disc = [cv.getStructuringElement](../../d4/d86/group__imgproc__filter.html#gac342a1bb6eabf6f55c803b09268e36dc "../../d4/d86/group__imgproc__filter.html#gac342a1bb6eabf6f55c803b09268e36dc")(cv.MORPH\_ELLIPSE,(5,5))
[cv.filter2D](../../d4/d86/group__imgproc__filter.html#ga27c049795ce870216ddfb366086b5a04 "../../d4/d86/group__imgproc__filter.html#ga27c049795ce870216ddfb366086b5a04")(dst,-1,disc,dst)

# threshold and binary AND
ret,thresh = [cv.threshold](../../d7/d1b/group__imgproc__misc.html#gae8a4a146d1ca78c626a53577199e9c57 "../../d7/d1b/group__imgproc__misc.html#gae8a4a146d1ca78c626a53577199e9c57")(dst,50,255,0)
thresh = [cv.merge](../../d2/de8/group__core__array.html#ga7d7b4d6c6ee504b30a20b1680029c7b4 "../../d2/de8/group__core__array.html#ga7d7b4d6c6ee504b30a20b1680029c7b4")((thresh,thresh,thresh))
res = [cv.bitwise\_and](../../d2/de8/group__core__array.html#ga60b4d04b251ba5eb1392c34425497e14 "../../d2/de8/group__core__array.html#ga60b4d04b251ba5eb1392c34425497e14")(target,thresh)

res = np.vstack((target,thresh,res))
[cv.imwrite](../../d4/da8/group__imgcodecs.html#ga8ac397bd09e48851665edbe12aa28f25 "../../d4/da8/group__imgcodecs.html#ga8ac397bd09e48851665edbe12aa28f25")('res.jpg',res)
[cv::bitwise\_and](../../d2/de8/group__core__array.html#ga60b4d04b251ba5eb1392c34425497e14 "../../d2/de8/group__core__array.html#ga60b4d04b251ba5eb1392c34425497e14")void bitwise\_and(InputArray src1, InputArray src2, OutputArray dst, InputArray mask=noArray())computes bitwise conjunction of the two arrays (dst = src1 & src2) Calculates the per-element bit-wis...
[cv::merge](../../d2/de8/group__core__array.html#ga7d7b4d6c6ee504b30a20b1680029c7b4 "../../d2/de8/group__core__array.html#ga7d7b4d6c6ee504b30a20b1680029c7b4")void merge(const Mat \*mv, size\_t count, OutputArray dst)Creates one multi-channel array out of several single-channel ones.
[cv::imwrite](../../d4/da8/group__imgcodecs.html#ga8ac397bd09e48851665edbe12aa28f25 "../../d4/da8/group__imgcodecs.html#ga8ac397bd09e48851665edbe12aa28f25")CV\_EXPORTS\_W bool imwrite(const String &filename, InputArray img, const std::vector< int > &params=std::vector< int >())Saves an image to a specified file.
[cv::calcBackProject](../../d6/dc7/group__imgproc__hist.html#ga3a0af640716b456c3d14af8aee12e3ca "../../d6/dc7/group__imgproc__hist.html#ga3a0af640716b456c3d14af8aee12e3ca")void calcBackProject(const Mat \*images, int nimages, const int \*channels, InputArray hist, OutputArray backProject, const float \*\*ranges, double scale=1, bool uniform=true)Calculates the back projection of a histogram.
 Below is one example I worked with. I used the region inside blue rectangle as sample object and I wanted to extract the full ground.

![](../../backproject_opencv.jpg)

image
# Additional Resources

1. "Indexing via color histograms", Swain, Michael J. , Third international conference on computer vision,1990.

# Exercises

