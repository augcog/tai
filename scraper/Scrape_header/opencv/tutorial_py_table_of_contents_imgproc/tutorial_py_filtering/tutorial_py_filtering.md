

OpenCV: Smoothing Images

 MathJax.Hub.Config({
 extensions: ["tex2jax.js", "TeX/AMSmath.js", "TeX/AMSsymbols.js"],
 jax: ["input/TeX","output/HTML-CSS"],
});
//<![CDATA[
MathJax.Hub.Config(
{
 TeX: {
 Macros: {
 matTT: [ "\\[ \\left|\\begin{array}{ccc} #1 & #2 & #3\\\\ #4 & #5 & #6\\\\ #7 & #8 & #9 \\end{array}\\right| \\]", 9],
 fork: ["\\left\\{ \\begin{array}{l l} #1 & \\mbox{#2}\\\\ #3 & \\mbox{#4}\\\\ \\end{array} \\right.", 4],
 forkthree: ["\\left\\{ \\begin{array}{l l} #1 & \\mbox{#2}\\\\ #3 & \\mbox{#4}\\\\ #5 & \\mbox{#6}\\\\ \\end{array} \\right.", 6],
 forkfour: ["\\left\\{ \\begin{array}{l l} #1 & \\mbox{#2}\\\\ #3 & \\mbox{#4}\\\\ #5 & \\mbox{#6}\\\\ #7 & \\mbox{#8}\\\\ \\end{array} \\right.", 8],
 vecthree: ["\\begin{bmatrix} #1\\\\ #2\\\\ #3 \\end{bmatrix}", 3],
 vecthreethree: ["\\begin{bmatrix} #1 & #2 & #3\\\\ #4 & #5 & #6\\\\ #7 & #8 & #9 \\end{bmatrix}", 9],
 cameramatrix: ["#1 = \\begin{bmatrix} f\_x & 0 & c\_x\\\\ 0 & f\_y & c\_y\\\\ 0 & 0 & 1 \\end{bmatrix}", 1],
 distcoeffs: ["(k\_1, k\_2, p\_1, p\_2[, k\_3[, k\_4, k\_5, k\_6 [, s\_1, s\_2, s\_3, s\_4[, \\tau\_x, \\tau\_y]]]]) \\text{ of 4, 5, 8, 12 or 14 elements}"],
 distcoeffsfisheye: ["(k\_1, k\_2, k\_3, k\_4)"],
 hdotsfor: ["\\dots", 1],
 mathbbm: ["\\mathbb{#1}", 1],
 bordermatrix: ["\\matrix{#1}", 1]
 }
 }
}
);
//]]>

 (function() {
 var cx = '002541620211387084530:kaexgxg7oxu';
 var gcse = document.createElement('script');
 gcse.type = 'text/javascript';
 gcse.async = true;
 gcse.src = 'https://cse.google.com/cse.js?cx=' + cx;
 var s = document.getElementsByTagName('script')[0];
 s.parentNode.insertBefore(gcse, s);
 })();

|  |  |
| --- | --- |
| Logo | OpenCV
 4.8.0-dev

Open Source Computer Vision |

var searchBox = new SearchBox("searchBox", "../../search",false,'Search');

$(function() {
 initMenu('../../',true,false,'search.php','Search');
 $(document).ready(function() { init\_search(); });
});

* [OpenCV-Python Tutorials](../../d6/d00/tutorial_py_root.html "../../d6/d00/tutorial_py_root.html")
* [Image Processing in OpenCV](../../d2/d96/tutorial_py_table_of_contents_imgproc.html "../../d2/d96/tutorial_py_table_of_contents_imgproc.html")

Smoothing Images  

## Goals

Learn to:

* Blur images with various low pass filters
* Apply custom-made filters to images (2D convolution)

## 2D Convolution ( Image Filtering )

As in one-dimensional signals, images also can be filtered with various low-pass filters (LPF), high-pass filters (HPF), etc. LPF helps in removing noise, blurring images, etc. HPF filters help in finding edges in images.

OpenCV provides a function **[cv.filter2D()](../../d4/d86/group__imgproc__filter.html#ga27c049795ce870216ddfb366086b5a04 "Convolves an image with the kernel. ")** to convolve a kernel with an image. As an example, we will try an averaging filter on an image. A 5x5 averaging filter kernel will look like the below:

\[K = \frac{1}{25} \begin{bmatrix} 1 & 1 & 1 & 1 & 1 \\ 1 & 1 & 1 & 1 & 1 \\ 1 & 1 & 1 & 1 & 1 \\ 1 & 1 & 1 & 1 & 1 \\ 1 & 1 & 1 & 1 & 1 \end{bmatrix}\]

The operation works like this: keep this kernel above a pixel, add all the 25 pixels below this kernel, take the average, and replace the central pixel with the new average value. This operation is continued for all the pixels in the image. Try this code and check the result: 

import numpy as npimport cv2 as cvfrom matplotlib import pyplot as pltimg = [cv.imread](../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56 "../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56")('opencv\_logo.png')assert img is not None, "file could not be read, check with os.path.exists()"kernel = np.ones((5,5),np.float32)/25dst = [cv.filter2D](../../d4/d86/group__imgproc__filter.html#ga27c049795ce870216ddfb366086b5a04 "../../d4/d86/group__imgproc__filter.html#ga27c049795ce870216ddfb366086b5a04")(img,-1,kernel)plt.subplot(121),plt.imshow(img),plt.title('Original')plt.xticks([]), plt.yticks([])plt.subplot(122),plt.imshow(dst),plt.title('Averaging')plt.xticks([]), plt.yticks([])plt.show() Result:

![filter.jpg](../../filter.jpg)

image
## Image Blurring (Image Smoothing)

Image blurring is achieved by convolving the image with a low-pass filter kernel. It is useful for removing noise. It actually removes high frequency content (eg: noise, edges) from the image. So edges are blurred a little bit in this operation (there are also blurring techniques which don't blur the edges). OpenCV provides four main types of blurring techniques.

### 1. Averaging

This is done by convolving an image with a normalized box filter. It simply takes the average of all the pixels under the kernel area and replaces the central element. This is done by the function **[cv.blur()](../../d4/d86/group__imgproc__filter.html#ga8c45db9afe636703801b0b2e440fce37 "Blurs an image using the normalized box filter. ")** or **[cv.boxFilter()](../../d4/d86/group__imgproc__filter.html#gad533230ebf2d42509547d514f7d3fbc3 "Blurs an image using the box filter. ")**. Check the docs for more details about the kernel. We should specify the width and height of the kernel. A 3x3 normalized box filter would look like the below:

\[K = \frac{1}{9} \begin{bmatrix} 1 & 1 & 1 \\ 1 & 1 & 1 \\ 1 & 1 & 1 \end{bmatrix}\]

NoteIf you don't want to use a normalized box filter, use **[cv.boxFilter()](../../d4/d86/group__imgproc__filter.html#gad533230ebf2d42509547d514f7d3fbc3 "Blurs an image using the box filter. ")**. Pass an argument normalize=False to the function.
Check a sample demo below with a kernel of 5x5 size: 

import cv2 as cvimport numpy as npfrom matplotlib import pyplot as pltimg = [cv.imread](../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56 "../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56")('opencv-logo-white.png')assert img is not None, "file could not be read, check with os.path.exists()"blur = [cv.blur](../../d4/d86/group__imgproc__filter.html#ga8c45db9afe636703801b0b2e440fce37 "../../d4/d86/group__imgproc__filter.html#ga8c45db9afe636703801b0b2e440fce37")(img,(5,5))plt.subplot(121),plt.imshow(img),plt.title('Original')plt.xticks([]), plt.yticks([])plt.subplot(122),plt.imshow(blur),plt.title('Blurred')plt.xticks([]), plt.yticks([])plt.show() Result:

![blur.jpg](../../blur.jpg)

image
### 2. Gaussian Blurring

In this method, instead of a box filter, a Gaussian kernel is used. It is done with the function, **[cv.GaussianBlur()](../../d4/d86/group__imgproc__filter.html#gaabe8c836e97159a9193fb0b11ac52cf1 "Blurs an image using a Gaussian filter. ")**. We should specify the width and height of the kernel which should be positive and odd. We also should specify the standard deviation in the X and Y directions, sigmaX and sigmaY respectively. If only sigmaX is specified, sigmaY is taken as the same as sigmaX. If both are given as zeros, they are calculated from the kernel size. Gaussian blurring is highly effective in removing Gaussian noise from an image.

If you want, you can create a Gaussian kernel with the function, **[cv.getGaussianKernel()](../../d4/d86/group__imgproc__filter.html#gac05a120c1ae92a6060dd0db190a61afa "Returns Gaussian filter coefficients. ")**.

The above code can be modified for Gaussian blurring: 

blur = [cv.GaussianBlur](../../d4/d86/group__imgproc__filter.html#gaabe8c836e97159a9193fb0b11ac52cf1 "../../d4/d86/group__imgproc__filter.html#gaabe8c836e97159a9193fb0b11ac52cf1")(img,(5,5),0) Result:

![gaussian.jpg](../../gaussian.jpg)

image
### 3. Median Blurring

Here, the function **[cv.medianBlur()](../../d4/d86/group__imgproc__filter.html#ga564869aa33e58769b4469101aac458f9 "Blurs an image using the median filter. ")** takes the median of all the pixels under the kernel area and the central element is replaced with this median value. This is highly effective against salt-and-pepper noise in an image. Interestingly, in the above filters, the central element is a newly calculated value which may be a pixel value in the image or a new value. But in median blurring, the central element is always replaced by some pixel value in the image. It reduces the noise effectively. Its kernel size should be a positive odd integer.

In this demo, I added a 50% noise to our original image and applied median blurring. Check the result: 

median = [cv.medianBlur](../../d4/d86/group__imgproc__filter.html#ga564869aa33e58769b4469101aac458f9 "../../d4/d86/group__imgproc__filter.html#ga564869aa33e58769b4469101aac458f9")(img,5) Result:

![median.jpg](../../median.jpg)

image
### 4. Bilateral Filtering

**[cv.bilateralFilter()](../../d4/d86/group__imgproc__filter.html#ga9d7064d478c95d60003cf839430737ed "Applies the bilateral filter to an image. ")** is highly effective in noise removal while keeping edges sharp. But the operation is slower compared to other filters. We already saw that a Gaussian filter takes the neighbourhood around the pixel and finds its Gaussian weighted average. This Gaussian filter is a function of space alone, that is, nearby pixels are considered while filtering. It doesn't consider whether pixels have almost the same intensity. It doesn't consider whether a pixel is an edge pixel or not. So it blurs the edges also, which we don't want to do.

Bilateral filtering also takes a Gaussian filter in space, but one more Gaussian filter which is a function of pixel difference. The Gaussian function of space makes sure that only nearby pixels are considered for blurring, while the Gaussian function of intensity difference makes sure that only those pixels with similar intensities to the central pixel are considered for blurring. So it preserves the edges since pixels at edges will have large intensity variation.

The below sample shows use of a bilateral filter (For details on arguments, visit docs). 

blur = [cv.bilateralFilter](../../d4/d86/group__imgproc__filter.html#ga9d7064d478c95d60003cf839430737ed "../../d4/d86/group__imgproc__filter.html#ga9d7064d478c95d60003cf839430737ed")(img,9,75,75) Result:

![bilateral.jpg](../../bilateral.jpg)

image
 See, the texture on the surface is gone, but the edges are still preserved.

## Additional Resources

1. Details about the [bilateral filtering](http://people.csail.mit.edu/sparis/bf_course/ "http://people.csail.mit.edu/sparis/bf_course/")

## Exercises

---

Generated on Wed Oct 4 2023 23:37:11 for OpenCV by  [![doxygen](../../doxygen.png)](http://www.doxygen.org/index.html "http://www.doxygen.org/index.html") 1.8.13

//<![CDATA[
addTutorialsButtons();
//]]>

