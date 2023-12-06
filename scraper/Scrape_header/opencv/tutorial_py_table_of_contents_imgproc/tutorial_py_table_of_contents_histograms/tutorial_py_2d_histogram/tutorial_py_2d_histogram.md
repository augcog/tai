

OpenCV: Histograms - 3 : 2D Histograms

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
* [Histograms in OpenCV](../../de/db2/tutorial_py_table_of_contents_histograms.html "../../de/db2/tutorial_py_table_of_contents_histograms.html")

Histograms - 3 : 2D Histograms  

## Goal

In this chapter, we will learn to find and plot 2D histograms. It will be helpful in coming chapters.

## Introduction

In the first article, we calculated and plotted one-dimensional histogram. It is called one-dimensional because we are taking only one feature into our consideration, ie grayscale intensity value of the pixel. But in two-dimensional histograms, you consider two features. Normally it is used for finding color histograms where two features are Hue & Saturation values of every pixel.

There is a python sample (samples/python/color\_histogram.py) already for finding color histograms. We will try to understand how to create such a color histogram, and it will be useful in understanding further topics like Histogram Back-Projection.

## 2D Histogram in OpenCV

It is quite simple and calculated using the same function, **[cv.calcHist()](../../d6/dc7/group__imgproc__hist.html#ga4b2b5fd75503ff9e6844cc4dcdaed35d "Calculates a histogram of a set of arrays. ")**. For color histograms, we need to convert the image from BGR to HSV. (Remember, for 1D histogram, we converted from BGR to Grayscale). For 2D histograms, its parameters will be modified as follows:

* **channels = [0,1]** *because we need to process both H and S plane.*
* **bins = [180,256]** *180 for H plane and 256 for S plane.*
* **range = [0,180,0,256]** *Hue value lies between 0 and 180 & Saturation lies between 0 and 256.*

Now check the code below: 

import numpy as npimport cv2 as cvimg = [cv.imread](../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56 "../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56")('home.jpg')assert img is not None, "file could not be read, check with os.path.exists()"hsv = [cv.cvtColor](../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab "../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab")(img,cv.COLOR\_BGR2HSV)hist = [cv.calcHist](../../d6/dc7/group__imgproc__hist.html#ga6ca1876785483836f72a77ced8ea759a "../../d6/dc7/group__imgproc__hist.html#ga6ca1876785483836f72a77ced8ea759a")([hsv], [0, 1], None, [180, 256], [0, 180, 0, 256]) That's it.

## 2D Histogram in Numpy

Numpy also provides a specific function for this : **np.histogram2d()**. (Remember, for 1D histogram we used **np.histogram()** ). 

import numpy as npimport cv2 as cvfrom matplotlib import pyplot as pltimg = [cv.imread](../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56 "../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56")('home.jpg')assert img is not None, "file could not be read, check with os.path.exists()"hsv = [cv.cvtColor](../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab "../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab")(img,cv.COLOR\_BGR2HSV)hist, xbins, ybins = np.histogram2d(h.ravel(),s.ravel(),[180,256],[[0,180],[0,256]]) First argument is H plane, second one is the S plane, third is number of bins for each and fourth is their range.

Now we can check how to plot this color histogram.

## Plotting 2D Histograms

### Method - 1 : Using [cv.imshow()](../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563 "Displays an image in the specified window. ")

The result we get is a two dimensional array of size 180x256. So we can show them as we do normally, using [cv.imshow()](../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563 "Displays an image in the specified window. ") function. It will be a grayscale image and it won't give much idea what colors are there, unless you know the Hue values of different colors.

### Method - 2 : Using Matplotlib

We can use **[matplotlib.pyplot.imshow()](../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563 "Displays an image in the specified window. ")** function to plot 2D histogram with different color maps. It gives us a much better idea about the different pixel density. But this also, doesn't gives us idea what color is there on a first look, unless you know the Hue values of different colors. Still I prefer this method. It is simple and better.

NoteWhile using this function, remember, interpolation flag should be nearest for better results.
Consider code: 

import numpy as npimport cv2 as cvfrom matplotlib import pyplot as pltimg = [cv.imread](../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56 "../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56")('home.jpg')assert img is not None, "file could not be read, check with os.path.exists()"hsv = [cv.cvtColor](../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab "../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab")(img,cv.COLOR\_BGR2HSV)hist = [cv.calcHist](../../d6/dc7/group__imgproc__hist.html#ga6ca1876785483836f72a77ced8ea759a "../../d6/dc7/group__imgproc__hist.html#ga6ca1876785483836f72a77ced8ea759a")( [hsv], [0, 1], None, [180, 256], [0, 180, 0, 256] )plt.imshow(hist,interpolation = 'nearest')plt.show() Below is the input image and its color histogram plot. X axis shows S values and Y axis shows Hue.

![2dhist_matplotlib.jpg](../../2dhist_matplotlib.jpg)

image
 In histogram, you can see some high values near H = 100 and S = 200. It corresponds to blue of sky. Similarly another peak can be seen near H = 25 and S = 100. It corresponds to yellow of the palace. You can verify it with any image editing tools like GIMP.

### Method 3 : OpenCV sample style !!

There is a sample code for color-histogram in OpenCV-Python2 samples (samples/python/color\_histogram.py). If you run the code, you can see the histogram shows the corresponding color also. Or simply it outputs a color coded histogram. Its result is very good (although you need to add extra bunch of lines).

In that code, the author created a color map in HSV. Then converted it into BGR. The resulting histogram image is multiplied with this color map. He also uses some preprocessing steps to remove small isolated pixels, resulting in a good histogram.

I leave it to the readers to run the code, analyze it and have your own hack arounds. Below is the output of that code for the same image as above:

![2dhist_opencv.jpg](../../2dhist_opencv.jpg)

image
 You can clearly see in the histogram what colors are present, blue is there, yellow is there, and some white due to chessboard is there. Nice !!!

## Additional Resources

## Exercises

---

Generated on Wed Oct 4 2023 23:37:11 for OpenCV by  [![doxygen](../../doxygen.png)](http://www.doxygen.org/index.html "http://www.doxygen.org/index.html") 1.8.13

//<![CDATA[
addTutorialsButtons();
//]]>

