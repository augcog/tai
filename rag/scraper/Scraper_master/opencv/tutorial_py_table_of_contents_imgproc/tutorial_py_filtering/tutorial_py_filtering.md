
# Goals

Learn to:

* Blur images with various low pass filters
* Apply custom-made filters to images (2D convolution)

# 2D Convolution ( Image Filtering )

As in one-dimensional signals, images also can be filtered with various low-pass filters (LPF), high-pass filters (HPF), etc. LPF helps in removing noise, blurring images, etc. HPF filters help in finding edges in images.

OpenCV provides a function **[cv.filter2D()](../../d4/d86/group__imgproc__filter.html#ga27c049795ce870216ddfb366086b5a04 "Convolves an image with the kernel.")** to convolve a kernel with an image. As an example, we will try an averaging filter on an image. A 5x5 averaging filter kernel will look like the below:

\[K = \frac{1}{25} \begin{bmatrix} 1 & 1 & 1 & 1 & 1 \\ 1 & 1 & 1 & 1 & 1 \\ 1 & 1 & 1 & 1 & 1 \\ 1 & 1 & 1 & 1 & 1 \\ 1 & 1 & 1 & 1 & 1 \end{bmatrix}\]

The operation works like this: keep this kernel above a pixel, add all the 25 pixels below this kernel, take the average, and replace the central pixel with the new average value. This operation is continued for all the pixels in the image. Try this code and check the result: 

import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt

img = [cv.imread](../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8 "../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8")('opencv\_logo.png')
assert img is not None, "file could not be read, check with os.path.exists()"

kernel = np.ones((5,5),np.float32)/25
dst = [cv.filter2D](../../d4/d86/group__imgproc__filter.html#ga27c049795ce870216ddfb366086b5a04 "../../d4/d86/group__imgproc__filter.html#ga27c049795ce870216ddfb366086b5a04")(img,-1,kernel)

plt.subplot(121),plt.imshow(img),plt.title('Original')
plt.xticks([]), plt.yticks([])
plt.subplot(122),plt.imshow(dst),plt.title('Averaging')
plt.xticks([]), plt.yticks([])
plt.show()
[cv::imread](../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8 "../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8")CV\_EXPORTS\_W Mat imread(const String &filename, int flags=IMREAD\_COLOR)Loads an image from a file.
[cv::filter2D](../../d4/d86/group__imgproc__filter.html#ga27c049795ce870216ddfb366086b5a04 "../../d4/d86/group__imgproc__filter.html#ga27c049795ce870216ddfb366086b5a04")void filter2D(InputArray src, OutputArray dst, int ddepth, InputArray kernel, Point anchor=Point(-1,-1), double delta=0, int borderType=BORDER\_DEFAULT)Convolves an image with the kernel.
 Result:

![](../../filter.jpg)

image
# Image Blurring (Image Smoothing)

Image blurring is achieved by convolving the image with a low-pass filter kernel. It is useful for removing noise. It actually removes high frequency content (eg: noise, edges) from the image. So edges are blurred a little bit in this operation (there are also blurring techniques which don't blur the edges). OpenCV provides four main types of blurring techniques.

## 1. Averaging

This is done by convolving an image with a normalized box filter. It simply takes the average of all the pixels under the kernel area and replaces the central element. This is done by the function **[cv.blur()](../../d4/d86/group__imgproc__filter.html#ga8c45db9afe636703801b0b2e440fce37 "Blurs an image using the normalized box filter.")** or **[cv.boxFilter()](../../d4/d86/group__imgproc__filter.html#gad533230ebf2d42509547d514f7d3fbc3 "Blurs an image using the box filter.")**. Check the docs for more details about the kernel. We should specify the width and height of the kernel. A 3x3 normalized box filter would look like the below:

\[K = \frac{1}{9} \begin{bmatrix} 1 & 1 & 1 \\ 1 & 1 & 1 \\ 1 & 1 & 1 \end{bmatrix}\]

NoteIf you don't want to use a normalized box filter, use **[cv.boxFilter()](../../d4/d86/group__imgproc__filter.html#gad533230ebf2d42509547d514f7d3fbc3 "Blurs an image using the box filter.")**. Pass an argument normalize=False to the function.
Check a sample demo below with a kernel of 5x5 size: 

import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt

img = [cv.imread](../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8 "../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8")('opencv-logo-white.png')
assert img is not None, "file could not be read, check with os.path.exists()"

blur = [cv.blur](../../d4/d86/group__imgproc__filter.html#ga8c45db9afe636703801b0b2e440fce37 "../../d4/d86/group__imgproc__filter.html#ga8c45db9afe636703801b0b2e440fce37")(img,(5,5))

plt.subplot(121),plt.imshow(img),plt.title('Original')
plt.xticks([]), plt.yticks([])
plt.subplot(122),plt.imshow(blur),plt.title('Blurred')
plt.xticks([]), plt.yticks([])
plt.show()
[cv::blur](../../d4/d86/group__imgproc__filter.html#ga8c45db9afe636703801b0b2e440fce37 "../../d4/d86/group__imgproc__filter.html#ga8c45db9afe636703801b0b2e440fce37")void blur(InputArray src, OutputArray dst, Size ksize, Point anchor=Point(-1,-1), int borderType=BORDER\_DEFAULT)Blurs an image using the normalized box filter.
 Result:

![](../../blur.jpg)

image
## 2. Gaussian Blurring

In this method, instead of a box filter, a Gaussian kernel is used. It is done with the function, **[cv.GaussianBlur()](../../d4/d86/group__imgproc__filter.html#gaabe8c836e97159a9193fb0b11ac52cf1 "Blurs an image using a Gaussian filter.")**. We should specify the width and height of the kernel which should be positive and odd. We also should specify the standard deviation in the X and Y directions, sigmaX and sigmaY respectively. If only sigmaX is specified, sigmaY is taken as the same as sigmaX. If both are given as zeros, they are calculated from the kernel size. Gaussian blurring is highly effective in removing Gaussian noise from an image.

If you want, you can create a Gaussian kernel with the function, **[cv.getGaussianKernel()](../../d4/d86/group__imgproc__filter.html#gac05a120c1ae92a6060dd0db190a61afa "Returns Gaussian filter coefficients.")**.

The above code can be modified for Gaussian blurring: 

blur = [cv.GaussianBlur](../../d4/d86/group__imgproc__filter.html#gaabe8c836e97159a9193fb0b11ac52cf1 "../../d4/d86/group__imgproc__filter.html#gaabe8c836e97159a9193fb0b11ac52cf1")(img,(5,5),0)
[cv::GaussianBlur](../../d4/d86/group__imgproc__filter.html#gaabe8c836e97159a9193fb0b11ac52cf1 "../../d4/d86/group__imgproc__filter.html#gaabe8c836e97159a9193fb0b11ac52cf1")void GaussianBlur(InputArray src, OutputArray dst, Size ksize, double sigmaX, double sigmaY=0, int borderType=BORDER\_DEFAULT)Blurs an image using a Gaussian filter.
 Result:

![](../../gaussian.jpg)

image
## 3. Median Blurring

Here, the function **[cv.medianBlur()](../../d4/d86/group__imgproc__filter.html#ga564869aa33e58769b4469101aac458f9 "Blurs an image using the median filter.")** takes the median of all the pixels under the kernel area and the central element is replaced with this median value. This is highly effective against salt-and-pepper noise in an image. Interestingly, in the above filters, the central element is a newly calculated value which may be a pixel value in the image or a new value. But in median blurring, the central element is always replaced by some pixel value in the image. It reduces the noise effectively. Its kernel size should be a positive odd integer.

In this demo, I added a 50% noise to our original image and applied median blurring. Check the result: 

median = [cv.medianBlur](../../d4/d86/group__imgproc__filter.html#ga564869aa33e58769b4469101aac458f9 "../../d4/d86/group__imgproc__filter.html#ga564869aa33e58769b4469101aac458f9")(img,5)
[cv::medianBlur](../../d4/d86/group__imgproc__filter.html#ga564869aa33e58769b4469101aac458f9 "../../d4/d86/group__imgproc__filter.html#ga564869aa33e58769b4469101aac458f9")void medianBlur(InputArray src, OutputArray dst, int ksize)Blurs an image using the median filter.
 Result:

![](../../median.jpg)

image
## 4. Bilateral Filtering

**[cv.bilateralFilter()](../../d4/d86/group__imgproc__filter.html#ga9d7064d478c95d60003cf839430737ed "Applies the bilateral filter to an image.")** is highly effective in noise removal while keeping edges sharp. But the operation is slower compared to other filters. We already saw that a Gaussian filter takes the neighbourhood around the pixel and finds its Gaussian weighted average. This Gaussian filter is a function of space alone, that is, nearby pixels are considered while filtering. It doesn't consider whether pixels have almost the same intensity. It doesn't consider whether a pixel is an edge pixel or not. So it blurs the edges also, which we don't want to do.

Bilateral filtering also takes a Gaussian filter in space, but one more Gaussian filter which is a function of pixel difference. The Gaussian function of space makes sure that only nearby pixels are considered for blurring, while the Gaussian function of intensity difference makes sure that only those pixels with similar intensities to the central pixel are considered for blurring. So it preserves the edges since pixels at edges will have large intensity variation.

The below sample shows use of a bilateral filter (For details on arguments, visit docs). 

blur = [cv.bilateralFilter](../../d4/d86/group__imgproc__filter.html#ga9d7064d478c95d60003cf839430737ed "../../d4/d86/group__imgproc__filter.html#ga9d7064d478c95d60003cf839430737ed")(img,9,75,75)
[cv::bilateralFilter](../../d4/d86/group__imgproc__filter.html#ga9d7064d478c95d60003cf839430737ed "../../d4/d86/group__imgproc__filter.html#ga9d7064d478c95d60003cf839430737ed")void bilateralFilter(InputArray src, OutputArray dst, int d, double sigmaColor, double sigmaSpace, int borderType=BORDER\_DEFAULT)Applies the bilateral filter to an image.
 Result:

![](../../bilateral.jpg)

image
See, the texture on the surface is gone, but the edges are still preserved.

# Additional Resources

1. Details about the [bilateral filtering](http://people.csail.mit.edu/sparis/bf_course/ "http://people.csail.mit.edu/sparis/bf_course/")

# Exercises

