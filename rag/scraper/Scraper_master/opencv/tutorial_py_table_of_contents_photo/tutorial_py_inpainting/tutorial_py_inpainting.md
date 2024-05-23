
# Goal

In this chapter,

* We will learn how to remove small noises, strokes etc in old photographs by a method called inpainting
* We will see inpainting functionalities in OpenCV.

# Basics

Most of you will have some old degraded photos at your home with some black spots, some strokes etc on it. Have you ever thought of restoring it back? We can't simply erase them in a paint tool because it is will simply replace black structures with white structures which is of no use. In these cases, a technique called image inpainting is used. The basic idea is simple: Replace those bad marks with its neighbouring pixels so that it looks like the neighbourhood. Consider the image shown below (taken from [Wikipedia](https://en.wikipedia.org/wiki/Inpainting "https://en.wikipedia.org/wiki/Inpainting")):

![](../../inpaint_basics.jpg)

image
Several algorithms were designed for this purpose and OpenCV provides two of them. Both can be accessed by the same function, **[cv.inpaint()](../../d7/d8b/group__photo__inpaint.html#gaedd30dfa0214fec4c88138b51d678085 "Restores the selected region in an image using the region neighborhood.")**

First algorithm is based on the paper \*\*"An Image Inpainting Technique Based on the Fast Marching
Method"\*\* by Alexandru Telea in 2004. It is based on Fast Marching Method. Consider a region in the image to be inpainted. Algorithm starts from the boundary of this region and goes inside the region gradually filling everything in the boundary first. It takes a small neighbourhood around the pixel on the neighbourhood to be inpainted. This pixel is replaced by normalized weighted sum of all the known pixels in the neighbourhood. Selection of the weights is an important matter. More weightage is given to those pixels lying near to the point, near to the normal of the boundary and those lying on the boundary contours. Once a pixel is inpainted, it moves to next nearest pixel using Fast Marching Method. FMM ensures those pixels near the known pixels are inpainted first, so that it just works like a manual heuristic operation. This algorithm is enabled by using the flag, [cv.INPAINT\_TELEA](../../d7/d8b/group__photo__inpaint.html#gga9007b81edae8e7ead89219b316c109fba892824c38e258feb5e72f308a358d52e "Use the algorithm proposed by Alexandru Telea .").

Second algorithm is based on the paper \*\*"Navier-Stokes, Fluid Dynamics, and Image and Video
Inpainting"\*\* by Bertalmio, Marcelo, Andrea L. Bertozzi, and Guillermo Sapiro in 2001. This algorithm is based on fluid dynamics and utilizes partial differential equations. Basic principle is heurisitic. It first travels along the edges from known regions to unknown regions (because edges are meant to be continuous). It continues isophotes (lines joining points with same intensity, just like contours joins points with same elevation) while matching gradient vectors at the boundary of the inpainting region. For this, some methods from fluid dynamics are used. Once they are obtained, color is filled to reduce minimum variance in that area. This algorithm is enabled by using the flag, [cv.INPAINT\_NS](../../d7/d8b/group__photo__inpaint.html#gga9007b81edae8e7ead89219b316c109fba05e763003a805e6c11c673a9f4ba7d07 "Use Navier-Stokes based method.").

# Code

We need to create a mask of same size as that of input image, where non-zero pixels corresponds to the area which is to be inpainted. Everything else is simple. My image is degraded with some black strokes (I added manually). I created a corresponding strokes with Paint tool. 

import numpy as np
import cv2 as cv

img = [cv.imread](../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8 "../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8")('messi\_2.jpg')
mask = [cv.imread](../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8 "../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8")('mask2.png', cv.IMREAD\_GRAYSCALE)

dst = [cv.inpaint](../../d7/d8b/group__photo__inpaint.html#gaedd30dfa0214fec4c88138b51d678085 "../../d7/d8b/group__photo__inpaint.html#gaedd30dfa0214fec4c88138b51d678085")(img,mask,3,cv.INPAINT\_TELEA)

[cv.imshow](../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563 "../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563")('dst',dst)
[cv.waitKey](../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7 "../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7")(0)
[cv.destroyAllWindows](../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481 "../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481")()
[cv::imshow](../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563 "../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563")void imshow(const String &winname, InputArray mat)Displays an image in the specified window.
[cv::waitKey](../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7 "../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7")int waitKey(int delay=0)Waits for a pressed key.
[cv::destroyAllWindows](../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481 "../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481")void destroyAllWindows()Destroys all of the HighGUI windows.
[cv::imread](../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8 "../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8")CV\_EXPORTS\_W Mat imread(const String &filename, int flags=IMREAD\_COLOR)Loads an image from a file.
[cv::inpaint](../../d7/d8b/group__photo__inpaint.html#gaedd30dfa0214fec4c88138b51d678085 "../../d7/d8b/group__photo__inpaint.html#gaedd30dfa0214fec4c88138b51d678085")void inpaint(InputArray src, InputArray inpaintMask, OutputArray dst, double inpaintRadius, int flags)Restores the selected region in an image using the region neighborhood.
 See the result below. First image shows degraded input. Second image is the mask. Third image is the result of first algorithm and last image is the result of second algorithm.

![](../../inpaint_result.jpg)

image
# Additional Resources

1. Bertalmio, Marcelo, Andrea L. Bertozzi, and Guillermo Sapiro. "Navier-stokes, fluid dynamics,
and image and video inpainting." In Computer Vision and Pattern Recognition, 2001. CVPR 2001. Proceedings of the 2001 IEEE Computer Society Conference on, vol. 1, pp. I-355. IEEE, 2001.
2. Telea, Alexandru. "An image inpainting technique based on the fast marching method." Journal of graphics tools 9.1 (2004): 23-34.

# Exercises

1. OpenCV comes with an interactive sample on inpainting, samples/python/inpaint.py, try it.
2. A few months ago, I watched a video on [Content-Aware Fill](https://www.youtube.com/watch?v=ZtoUiplKa2A "https://www.youtube.com/watch?v=ZtoUiplKa2A"), an advanced inpainting technique used in Adobe Photoshop. On further search, I was able to find that same technique is already there in GIMP with different name, "Resynthesizer" (You need to install separate plugin). I am sure you will enjoy the technique.

