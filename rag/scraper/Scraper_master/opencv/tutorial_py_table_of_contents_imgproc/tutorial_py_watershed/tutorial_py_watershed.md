
# Goal

In this chapter,

* We will learn to use marker-based image segmentation using watershed algorithm
* We will see: **[cv.watershed()](../../d3/d47/group__imgproc__segmentation.html#ga3267243e4d3f95165d55a618c65ac6e1 "Performs a marker-based image segmentation using the watershed algorithm.")**

# Theory

Any grayscale image can be viewed as a topographic surface where high intensity denotes peaks and hills while low intensity denotes valleys. You start filling every isolated valleys (local minima) with different colored water (labels). As the water rises, depending on the peaks (gradients) nearby, water from different valleys, obviously with different colors will start to merge. To avoid that, you build barriers in the locations where water merges. You continue the work of filling water and building barriers until all the peaks are under water. Then the barriers you created gives you the segmentation result. This is the "philosophy" behind the watershed. You can visit the [CMM webpage on watershed](http://cmm.ensmp.fr/~beucher/wtshed.html "http://cmm.ensmp.fr/~beucher/wtshed.html") to understand it with the help of some animations.

But this approach gives you oversegmented result due to noise or any other irregularities in the image. So OpenCV implemented a marker-based watershed algorithm where you specify which are all valley points are to be merged and which are not. It is an interactive image segmentation. What we do is to give different labels for our object we know. Label the region which we are sure of being the foreground or object with one color (or intensity), label the region which we are sure of being background or non-object with another color and finally the region which we are not sure of anything, label it with 0. That is our marker. Then apply watershed algorithm. Then our marker will be updated with the labels we gave, and the boundaries of objects will have a value of -1.

# Code

Below we will see an example on how to use the Distance Transform along with watershed to segment mutually touching objects.

Consider the coins image below, the coins are touching each other. Even if you threshold it, it will be touching each other.

![](../../water_coins.jpg)

image
We start with finding an approximate estimate of the coins. For that, we can use the Otsu's binarization. 

import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt

img = [cv.imread](../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8 "../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8")('coins.png')
assert img is not None, "file could not be read, check with os.path.exists()"
gray = [cv.cvtColor](../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab "../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab")(img,cv.COLOR\_BGR2GRAY)
ret, thresh = [cv.threshold](../../d7/d1b/group__imgproc__misc.html#gae8a4a146d1ca78c626a53577199e9c57 "../../d7/d1b/group__imgproc__misc.html#gae8a4a146d1ca78c626a53577199e9c57")(gray,0,255,cv.THRESH\_BINARY\_INV+cv.THRESH\_OTSU)
[cv::imread](../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8 "../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8")CV\_EXPORTS\_W Mat imread(const String &filename, int flags=IMREAD\_COLOR)Loads an image from a file.
[cv::cvtColor](../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab "../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab")void cvtColor(InputArray src, OutputArray dst, int code, int dstCn=0)Converts an image from one color space to another.
[cv::threshold](../../d7/d1b/group__imgproc__misc.html#gae8a4a146d1ca78c626a53577199e9c57 "../../d7/d1b/group__imgproc__misc.html#gae8a4a146d1ca78c626a53577199e9c57")double threshold(InputArray src, OutputArray dst, double thresh, double maxval, int type)Applies a fixed-level threshold to each array element.
 Result:

![](../../water_thresh.jpg)

image
Now we need to remove any small white noises in the image. For that we can use morphological opening. To remove any small holes in the object, we can use morphological closing. So, now we know for sure that region near to center of objects are foreground and region much away from the object are background. Only region we are not sure is the boundary region of coins.

So we need to extract the area which we are sure they are coins. Erosion removes the boundary pixels. So whatever remaining, we can be sure it is coin. That would work if objects were not touching each other. But since they are touching each other, another good option would be to find the distance transform and apply a proper threshold. Next we need to find the area which we are sure they are not coins. For that, we dilate the result. Dilation increases object boundary to background. This way, we can make sure whatever region in background in result is really a background, since boundary region is removed. See the image below.

![](../../water_fgbg.jpg)

image
The remaining regions are those which we don't have any idea, whether it is coins or background. Watershed algorithm should find it. These areas are normally around the boundaries of coins where foreground and background meet (Or even two different coins meet). We call it border. It can be obtained from subtracting sure\_fg area from sure\_bg area. 

# noise removal
kernel = np.ones((3,3),np.uint8)
opening = [cv.morphologyEx](../../d4/d86/group__imgproc__filter.html#ga67493776e3ad1a3df63883829375201f "../../d4/d86/group__imgproc__filter.html#ga67493776e3ad1a3df63883829375201f")(thresh,cv.MORPH\_OPEN,kernel, iterations = 2)

# sure background area
sure\_bg = [cv.dilate](../../d4/d86/group__imgproc__filter.html#ga4ff0f3318642c4f469d0e11f242f3b6c "../../d4/d86/group__imgproc__filter.html#ga4ff0f3318642c4f469d0e11f242f3b6c")(opening,kernel,iterations=3)

# Finding sure foreground area
dist\_transform = [cv.distanceTransform](../../d7/d1b/group__imgproc__misc.html#ga8a0b7fdfcb7a13dde018988ba3a43042 "../../d7/d1b/group__imgproc__misc.html#ga8a0b7fdfcb7a13dde018988ba3a43042")(opening,cv.DIST\_L2,5)
ret, sure\_fg = [cv.threshold](../../d7/d1b/group__imgproc__misc.html#gae8a4a146d1ca78c626a53577199e9c57 "../../d7/d1b/group__imgproc__misc.html#gae8a4a146d1ca78c626a53577199e9c57")(dist\_transform,0.7\*dist\_transform.max(),255,0)

# Finding unknown region
sure\_fg = np.uint8(sure\_fg)
unknown = [cv.subtract](../../d2/de8/group__core__array.html#gaa0f00d98b4b5edeaeb7b8333b2de353b "../../d2/de8/group__core__array.html#gaa0f00d98b4b5edeaeb7b8333b2de353b")(sure\_bg,sure\_fg)
[cv::subtract](../../d2/de8/group__core__array.html#gaa0f00d98b4b5edeaeb7b8333b2de353b "../../d2/de8/group__core__array.html#gaa0f00d98b4b5edeaeb7b8333b2de353b")void subtract(InputArray src1, InputArray src2, OutputArray dst, InputArray mask=noArray(), int dtype=-1)Calculates the per-element difference between two arrays or array and a scalar.
[cv::dilate](../../d4/d86/group__imgproc__filter.html#ga4ff0f3318642c4f469d0e11f242f3b6c "../../d4/d86/group__imgproc__filter.html#ga4ff0f3318642c4f469d0e11f242f3b6c")void dilate(InputArray src, OutputArray dst, InputArray kernel, Point anchor=Point(-1,-1), int iterations=1, int borderType=BORDER\_CONSTANT, const Scalar &borderValue=morphologyDefaultBorderValue())Dilates an image by using a specific structuring element.
[cv::morphologyEx](../../d4/d86/group__imgproc__filter.html#ga67493776e3ad1a3df63883829375201f "../../d4/d86/group__imgproc__filter.html#ga67493776e3ad1a3df63883829375201f")void morphologyEx(InputArray src, OutputArray dst, int op, InputArray kernel, Point anchor=Point(-1,-1), int iterations=1, int borderType=BORDER\_CONSTANT, const Scalar &borderValue=morphologyDefaultBorderValue())Performs advanced morphological transformations.
[cv::distanceTransform](../../d7/d1b/group__imgproc__misc.html#ga8a0b7fdfcb7a13dde018988ba3a43042 "../../d7/d1b/group__imgproc__misc.html#ga8a0b7fdfcb7a13dde018988ba3a43042")void distanceTransform(InputArray src, OutputArray dst, OutputArray labels, int distanceType, int maskSize, int labelType=DIST\_LABEL\_CCOMP)Calculates the distance to the closest zero pixel for each pixel of the source image.
 See the result. In the thresholded image, we get some regions of coins which we are sure of coins and they are detached now. (In some cases, you may be interested in only foreground segmentation, not in separating the mutually touching objects. In that case, you need not use distance transform, just erosion is sufficient. Erosion is just another method to extract sure foreground area, that's all.)

![](../../water_dt.jpg)

image
Now we know for sure which are region of coins, which are background and all. So we create marker (it is an array of same size as that of original image, but with int32 datatype) and label the regions inside it. The regions we know for sure (whether foreground or background) are labelled with any positive integers, but different integers, and the area we don't know for sure are just left as zero. For this we use **[cv.connectedComponents()](../../d3/dc0/group__imgproc__shape.html#gaedef8c7340499ca391d459122e51bef5 "computes the connected components labeled image of boolean image")**. It labels background of the image with 0, then other objects are labelled with integers starting from 1.

But we know that if background is marked with 0, watershed will consider it as unknown area. So we want to mark it with different integer. Instead, we will mark unknown region, defined by unknown, with 0. 

# Marker labelling
ret, markers = [cv.connectedComponents](../../d3/dc0/group__imgproc__shape.html#gaedef8c7340499ca391d459122e51bef5 "../../d3/dc0/group__imgproc__shape.html#gaedef8c7340499ca391d459122e51bef5")(sure\_fg)

# Add one to all labels so that sure background is not 0, but 1
markers = markers+1

# Now, mark the region of unknown with zero
markers[unknown==255] = 0
[cv::connectedComponents](../../d3/dc0/group__imgproc__shape.html#gaedef8c7340499ca391d459122e51bef5 "../../d3/dc0/group__imgproc__shape.html#gaedef8c7340499ca391d459122e51bef5")int connectedComponents(InputArray image, OutputArray labels, int connectivity, int ltype, int ccltype)computes the connected components labeled image of boolean image
 See the result shown in JET colormap. The dark blue region shows unknown region. Sure coins are colored with different values. Remaining area which are sure background are shown in lighter blue compared to unknown region.

![](../../water_marker.jpg)

image
Now our marker is ready. It is time for final step, apply watershed. Then marker image will be modified. The boundary region will be marked with -1. 

markers = [cv.watershed](../../d3/d47/group__imgproc__segmentation.html#ga3267243e4d3f95165d55a618c65ac6e1 "../../d3/d47/group__imgproc__segmentation.html#ga3267243e4d3f95165d55a618c65ac6e1")(img,markers)
img[markers == -1] = [255,0,0]
[cv::watershed](../../d3/d47/group__imgproc__segmentation.html#ga3267243e4d3f95165d55a618c65ac6e1 "../../d3/d47/group__imgproc__segmentation.html#ga3267243e4d3f95165d55a618c65ac6e1")void watershed(InputArray image, InputOutputArray markers)Performs a marker-based image segmentation using the watershed algorithm.
 See the result below. For some coins, the region where they touch are segmented properly and for some, they are not.

![](../../water_result.jpg)

image
# Additional Resources

1. CMM page on [Watershed Transformation](http://cmm.ensmp.fr/~beucher/wtshed.html "http://cmm.ensmp.fr/~beucher/wtshed.html")

# Exercises

1. OpenCV samples has an interactive sample on watershed segmentation, watershed.py. Run it, Enjoy it, then learn it.

