

OpenCV: Morphological Transformations

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

Morphological Transformations  

## Goal

In this chapter,

* We will learn different morphological operations like Erosion, Dilation, Opening, Closing etc.
* We will see different functions like : **[cv.erode()](../../d4/d86/group__imgproc__filter.html#gaeb1e0c1033e3f6b891a25d0511362aeb "Erodes an image by using a specific structuring element. ")**, **[cv.dilate()](../../d4/d86/group__imgproc__filter.html#ga4ff0f3318642c4f469d0e11f242f3b6c "Dilates an image by using a specific structuring element. ")**, **[cv.morphologyEx()](../../d4/d86/group__imgproc__filter.html#ga67493776e3ad1a3df63883829375201f "Performs advanced morphological transformations. ")** etc.

## Theory

Morphological transformations are some simple operations based on the image shape. It is normally performed on binary images. It needs two inputs, one is our original image, second one is called **structuring element** or **kernel** which decides the nature of operation. Two basic morphological operators are Erosion and Dilation. Then its variant forms like Opening, Closing, Gradient etc also comes into play. We will see them one-by-one with help of following image:

![j.png](../../j.png)

image
### 1. Erosion

The basic idea of erosion is just like soil erosion only, it erodes away the boundaries of foreground object (Always try to keep foreground in white). So what it does? The kernel slides through the image (as in 2D convolution). A pixel in the original image (either 1 or 0) will be considered 1 only if all the pixels under the kernel is 1, otherwise it is eroded (made to zero).

So what happends is that, all the pixels near boundary will be discarded depending upon the size of kernel. So the thickness or size of the foreground object decreases or simply white region decreases in the image. It is useful for removing small white noises (as we have seen in colorspace chapter), detach two connected objects etc.

Here, as an example, I would use a 5x5 kernel with full of ones. Let's see it how it works: 

import cv2 as cvimport numpy as npimg = [cv.imread](../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56 "../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56")('j.png', cv.IMREAD\_GRAYSCALE)assert img is not None, "file could not be read, check with os.path.exists()"kernel = np.ones((5,5),np.uint8)erosion = [cv.erode](../../d4/d86/group__imgproc__filter.html#gaeb1e0c1033e3f6b891a25d0511362aeb "../../d4/d86/group__imgproc__filter.html#gaeb1e0c1033e3f6b891a25d0511362aeb")(img,kernel,iterations = 1) Result:

![erosion.png](../../erosion.png)

image
### 2. Dilation

It is just opposite of erosion. Here, a pixel element is '1' if at least one pixel under the kernel is '1'. So it increases the white region in the image or size of foreground object increases. Normally, in cases like noise removal, erosion is followed by dilation. Because, erosion removes white noises, but it also shrinks our object. So we dilate it. Since noise is gone, they won't come back, but our object area increases. It is also useful in joining broken parts of an object. 

dilation = [cv.dilate](../../d4/d86/group__imgproc__filter.html#ga4ff0f3318642c4f469d0e11f242f3b6c "../../d4/d86/group__imgproc__filter.html#ga4ff0f3318642c4f469d0e11f242f3b6c")(img,kernel,iterations = 1) Result:

![dilation.png](../../dilation.png)

image
### 3. Opening

Opening is just another name of **erosion followed by dilation**. It is useful in removing noise, as we explained above. Here we use the function, **[cv.morphologyEx()](../../d4/d86/group__imgproc__filter.html#ga67493776e3ad1a3df63883829375201f "Performs advanced morphological transformations. ")** 

opening = [cv.morphologyEx](../../d4/d86/group__imgproc__filter.html#ga67493776e3ad1a3df63883829375201f "../../d4/d86/group__imgproc__filter.html#ga67493776e3ad1a3df63883829375201f")(img, cv.MORPH\_OPEN, kernel) Result:

![opening.png](../../opening.png)

image
### 4. Closing

Closing is reverse of Opening, **Dilation followed by Erosion**. It is useful in closing small holes inside the foreground objects, or small black points on the object. 

closing = [cv.morphologyEx](../../d4/d86/group__imgproc__filter.html#ga67493776e3ad1a3df63883829375201f "../../d4/d86/group__imgproc__filter.html#ga67493776e3ad1a3df63883829375201f")(img, cv.MORPH\_CLOSE, kernel) Result:

![closing.png](../../closing.png)

image
### 5. Morphological Gradient

It is the difference between dilation and erosion of an image.

The result will look like the outline of the object. 

gradient = [cv.morphologyEx](../../d4/d86/group__imgproc__filter.html#ga67493776e3ad1a3df63883829375201f "../../d4/d86/group__imgproc__filter.html#ga67493776e3ad1a3df63883829375201f")(img, cv.MORPH\_GRADIENT, kernel) Result:

![gradient.png](../../gradient.png)

image
### 6. Top Hat

It is the difference between input image and Opening of the image. Below example is done for a 9x9 kernel. 

tophat = [cv.morphologyEx](../../d4/d86/group__imgproc__filter.html#ga67493776e3ad1a3df63883829375201f "../../d4/d86/group__imgproc__filter.html#ga67493776e3ad1a3df63883829375201f")(img, cv.MORPH\_TOPHAT, kernel) Result:

![tophat.png](../../tophat.png)

image
### 7. Black Hat

It is the difference between the closing of the input image and input image. 

blackhat = [cv.morphologyEx](../../d4/d86/group__imgproc__filter.html#ga67493776e3ad1a3df63883829375201f "../../d4/d86/group__imgproc__filter.html#ga67493776e3ad1a3df63883829375201f")(img, cv.MORPH\_BLACKHAT, kernel) Result:

![blackhat.png](../../blackhat.png)

image
## Structuring Element

We manually created a structuring elements in the previous examples with help of Numpy. It is rectangular shape. But in some cases, you may need elliptical/circular shaped kernels. So for this purpose, OpenCV has a function, **[cv.getStructuringElement()](../../d4/d86/group__imgproc__filter.html#gac342a1bb6eabf6f55c803b09268e36dc "Returns a structuring element of the specified size and shape for morphological operations. ")**. You just pass the shape and size of the kernel, you get the desired kernel. 

# Rectangular Kernel>>> [cv.getStructuringElement](../../d4/d86/group__imgproc__filter.html#gac342a1bb6eabf6f55c803b09268e36dc "../../d4/d86/group__imgproc__filter.html#gac342a1bb6eabf6f55c803b09268e36dc")(cv.MORPH\_RECT,(5,5))array([[1, 1, 1, 1, 1], [1, 1, 1, 1, 1], [1, 1, 1, 1, 1], [1, 1, 1, 1, 1], [1, 1, 1, 1, 1]], dtype=uint8)# Elliptical Kernel>>> [cv.getStructuringElement](../../d4/d86/group__imgproc__filter.html#gac342a1bb6eabf6f55c803b09268e36dc "../../d4/d86/group__imgproc__filter.html#gac342a1bb6eabf6f55c803b09268e36dc")(cv.MORPH\_ELLIPSE,(5,5))array([[0, 0, 1, 0, 0], [1, 1, 1, 1, 1], [1, 1, 1, 1, 1], [1, 1, 1, 1, 1], [0, 0, 1, 0, 0]], dtype=uint8)# Cross-shaped Kernel>>> [cv.getStructuringElement](../../d4/d86/group__imgproc__filter.html#gac342a1bb6eabf6f55c803b09268e36dc "../../d4/d86/group__imgproc__filter.html#gac342a1bb6eabf6f55c803b09268e36dc")(cv.MORPH\_CROSS,(5,5))array([[0, 0, 1, 0, 0], [0, 0, 1, 0, 0], [1, 1, 1, 1, 1], [0, 0, 1, 0, 0], [0, 0, 1, 0, 0]], dtype=uint8) ## Additional Resources

1. [Morphological Operations](http://homepages.inf.ed.ac.uk/rbf/HIPR2/morops.htm "http://homepages.inf.ed.ac.uk/rbf/HIPR2/morops.htm") at HIPR2

## Exercises

---

Generated on Wed Oct 4 2023 23:37:11 for OpenCV by  [![doxygen](../../doxygen.png)](http://www.doxygen.org/index.html "http://www.doxygen.org/index.html") 1.8.13

//<![CDATA[
addTutorialsButtons();
//]]>

