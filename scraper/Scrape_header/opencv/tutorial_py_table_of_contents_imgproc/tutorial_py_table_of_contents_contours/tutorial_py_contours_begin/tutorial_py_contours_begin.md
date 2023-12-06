

OpenCV: Contours : Getting Started

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
* [Contours in OpenCV](../../d3/d05/tutorial_py_table_of_contents_contours.html "../../d3/d05/tutorial_py_table_of_contents_contours.html")

Contours : Getting Started  

**Next Tutorial:** [Contour Features](../../dd/d49/tutorial_py_contour_features.html "../../dd/d49/tutorial_py_contour_features.html")

## Goal

* Understand what contours are.
* Learn to find contours, draw contours etc
* You will see these functions : **[cv.findContours()](../../d3/dc0/group__imgproc__shape.html#gadf1ad6a0b82947fa1fe3c3d497f260e0 "Finds contours in a binary image. ")**, **[cv.drawContours()](../../d6/d6e/group__imgproc__draw.html#ga746c0625f1781f1ffc9056259103edbc "Draws contours outlines or filled contours. ")**

## What are contours?

Contours can be explained simply as a curve joining all the continuous points (along the boundary), having same color or intensity. The contours are a useful tool for shape analysis and object detection and recognition.

* For better accuracy, use binary images. So before finding contours, apply threshold or canny edge detection.
* Since OpenCV 3.2, [findContours()](../../d3/dc0/group__imgproc__shape.html#gadf1ad6a0b82947fa1fe3c3d497f260e0 "Finds contours in a binary image. ") no longer modifies the source image.
* In OpenCV, finding contours is like finding white object from black background. So remember, object to be found should be white and background should be black.

Let's see how to find contours of a binary image: 

import numpy as npimport cv2 as cvim = [cv.imread](../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56 "../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56")('test.jpg')assert im is not None, "file could not be read, check with os.path.exists()"imgray = [cv.cvtColor](../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab "../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab")(im, cv.COLOR\_BGR2GRAY)ret, thresh = [cv.threshold](../../d7/d1b/group__imgproc__misc.html#gae8a4a146d1ca78c626a53577199e9c57 "../../d7/d1b/group__imgproc__misc.html#gae8a4a146d1ca78c626a53577199e9c57")(imgray, 127, 255, 0)contours, hierarchy = [cv.findContours](../../d3/dc0/group__imgproc__shape.html#gae4156f04053c44f886e387cff0ef6e08 "../../d3/dc0/group__imgproc__shape.html#gae4156f04053c44f886e387cff0ef6e08")(thresh, cv.RETR\_TREE, cv.CHAIN\_APPROX\_SIMPLE) See, there are three arguments in **[cv.findContours()](../../d3/dc0/group__imgproc__shape.html#gadf1ad6a0b82947fa1fe3c3d497f260e0 "Finds contours in a binary image. ")** function, first one is source image, second is contour retrieval mode, third is contour approximation method. And it outputs the contours and hierarchy. Contours is a Python list of all the contours in the image. Each individual contour is a Numpy array of (x,y) coordinates of boundary points of the object.

NoteWe will discuss second and third arguments and about hierarchy in details later. Until then, the values given to them in code sample will work fine for all images.
## How to draw the contours?

To draw the contours, [cv.drawContours](../../d6/d6e/group__imgproc__draw.html#ga746c0625f1781f1ffc9056259103edbc "Draws contours outlines or filled contours. ") function is used. It can also be used to draw any shape provided you have its boundary points. Its first argument is source image, second argument is the contours which should be passed as a Python list, third argument is index of contours (useful when drawing individual contour. To draw all contours, pass -1) and remaining arguments are color, thickness etc.

* To draw all the contours in an image: [cv.drawContours](../../d6/d6e/group__imgproc__draw.html#ga746c0625f1781f1ffc9056259103edbc "../../d6/d6e/group__imgproc__draw.html#ga746c0625f1781f1ffc9056259103edbc")(img, contours, -1, (0,255,0), 3)
* To draw an individual contour, say 4th contour: [cv.drawContours](../../d6/d6e/group__imgproc__draw.html#ga746c0625f1781f1ffc9056259103edbc "../../d6/d6e/group__imgproc__draw.html#ga746c0625f1781f1ffc9056259103edbc")(img, contours, 3, (0,255,0), 3)
* But most of the time, below method will be useful: cnt = contours[4][cv.drawContours](../../d6/d6e/group__imgproc__draw.html#ga746c0625f1781f1ffc9056259103edbc "../../d6/d6e/group__imgproc__draw.html#ga746c0625f1781f1ffc9056259103edbc")(img, [cnt], 0, (0,255,0), 3)

NoteLast two methods are same, but when you go forward, you will see last one is more useful.
# Contour Approximation Method

This is the third argument in [cv.findContours](../../d3/dc0/group__imgproc__shape.html#gadf1ad6a0b82947fa1fe3c3d497f260e0 "Finds contours in a binary image. ") function. What does it denote actually?

Above, we told that contours are the boundaries of a shape with same intensity. It stores the (x,y) coordinates of the boundary of a shape. But does it store all the coordinates ? That is specified by this contour approximation method.

If you pass [cv.CHAIN\_APPROX\_NONE](../../d3/dc0/group__imgproc__shape.html#gga4303f45752694956374734a03c54d5ffaf7d9a3582d021d5dadcb0e37201a62f8 "../../d3/dc0/group__imgproc__shape.html#gga4303f45752694956374734a03c54d5ffaf7d9a3582d021d5dadcb0e37201a62f8"), all the boundary points are stored. But actually do we need all the points? For eg, you found the contour of a straight line. Do you need all the points on the line to represent that line? No, we need just two end points of that line. This is what [cv.CHAIN\_APPROX\_SIMPLE](../../d3/dc0/group__imgproc__shape.html#gga4303f45752694956374734a03c54d5ffa5f2883048e654999209f88ba04c302f5 "../../d3/dc0/group__imgproc__shape.html#gga4303f45752694956374734a03c54d5ffa5f2883048e654999209f88ba04c302f5") does. It removes all redundant points and compresses the contour, thereby saving memory.

Below image of a rectangle demonstrate this technique. Just draw a circle on all the coordinates in the contour array (drawn in blue color). First image shows points I got with [cv.CHAIN\_APPROX\_NONE](../../d3/dc0/group__imgproc__shape.html#gga4303f45752694956374734a03c54d5ffaf7d9a3582d021d5dadcb0e37201a62f8 "../../d3/dc0/group__imgproc__shape.html#gga4303f45752694956374734a03c54d5ffaf7d9a3582d021d5dadcb0e37201a62f8") (734 points) and second image shows the one with [cv.CHAIN\_APPROX\_SIMPLE](../../d3/dc0/group__imgproc__shape.html#gga4303f45752694956374734a03c54d5ffa5f2883048e654999209f88ba04c302f5 "../../d3/dc0/group__imgproc__shape.html#gga4303f45752694956374734a03c54d5ffa5f2883048e654999209f88ba04c302f5") (only 4 points). See, how much memory it saves!!!

![none.jpg](../../none.jpg)

image
## Additional Resources

## Exercises

---

Generated on Wed Oct 4 2023 23:37:11 for OpenCV by  [![doxygen](../../doxygen.png)](http://www.doxygen.org/index.html "http://www.doxygen.org/index.html") 1.8.13

//<![CDATA[
addTutorialsButtons();
//]]>

