

OpenCV: Hough Line Transform

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

Hough Line Transform  

## Goal

In this chapter,

* We will understand the concept of the Hough Transform.
* We will see how to use it to detect lines in an image.
* We will see the following functions: **[cv.HoughLines()](../../dd/d1a/group__imgproc__feature.html#ga46b4e588934f6c8dfd509cc6e0e4545a "Finds lines in a binary image using the standard Hough transform. ")**, **[cv.HoughLinesP()](../../dd/d1a/group__imgproc__feature.html#ga8618180a5948286384e3b7ca02f6feeb "Finds line segments in a binary image using the probabilistic Hough transform. ")**

## Theory

The Hough Transform is a popular technique to detect any shape, if you can represent that shape in a mathematical form. It can detect the shape even if it is broken or distorted a little bit. We will see how it works for a line.

A line can be represented as \(y = mx+c\) or in a parametric form, as \(\rho = x \cos \theta + y \sin \theta\) where \(\rho\) is the perpendicular distance from the origin to the line, and \(\theta\) is the angle formed by this perpendicular line and the horizontal axis measured in counter-clockwise (That direction varies on how you represent the coordinate system. This representation is used in OpenCV). Check the image below:

houghlines1.svg

image
 So if the line is passing below the origin, it will have a positive rho and an angle less than 180. If it is going above the origin, instead of taking an angle greater than 180, the angle is taken less than 180, and rho is taken negative. Any vertical line will have 0 degree and horizontal lines will have 90 degree.

Now let's see how the Hough Transform works for lines. Any line can be represented in these two terms, \((\rho, \theta)\). So first it creates a 2D array or accumulator (to hold the values of the two parameters) and it is set to 0 initially. Let rows denote the \(\rho\) and columns denote the \(\theta\). Size of array depends on the accuracy you need. Suppose you want the accuracy of angles to be 1 degree, you will need 180 columns. For \(\rho\), the maximum distance possible is the diagonal length of the image. So taking one pixel accuracy, the number of rows can be the diagonal length of the image.

Consider a 100x100 image with a horizontal line at the middle. Take the first point of the line. You know its (x,y) values. Now in the line equation, put the values \(\theta = 0,1,2,....,180\) and check the \(\rho\) you get. For every \((\rho, \theta)\) pair, you increment value by one in our accumulator in its corresponding \((\rho, \theta)\) cells. So now in accumulator, the cell (50,90) = 1 along with some other cells.

Now take the second point on the line. Do the same as above. Increment the values in the cells corresponding to `(rho, theta)` you got. This time, the cell (50,90) = 2. What you actually do is voting the \((\rho, \theta)\) values. You continue this process for every point on the line. At each point, the cell (50,90) will be incremented or voted up, while other cells may or may not be voted up. This way, at the end, the cell (50,90) will have maximum votes. So if you search the accumulator for maximum votes, you get the value (50,90) which says, there is a line in this image at a distance 50 from the origin and at angle 90 degrees. It is well shown in the below animation (Image Courtesy: [Amos Storkey](http://homepages.inf.ed.ac.uk/amos/hough.html "http://homepages.inf.ed.ac.uk/amos/hough.html") )

![houghlinesdemo.gif](../../houghlinesdemo.gif)

This is how hough transform works for lines. It is simple, and may be you can implement it using Numpy on your own. Below is an image which shows the accumulator. Bright spots at some locations denote they are the parameters of possible lines in the image. (Image courtesy: [Wikipedia](https://en.wikipedia.org/wiki/Hough_transform "https://en.wikipedia.org/wiki/Hough_transform") )

![houghlines2.jpg](../../houghlines2.jpg)

# Hough Transform in OpenCV

Everything explained above is encapsulated in the OpenCV function, **[cv.HoughLines()](../../dd/d1a/group__imgproc__feature.html#ga46b4e588934f6c8dfd509cc6e0e4545a "Finds lines in a binary image using the standard Hough transform. ")**. It simply returns an array of :math:(rho, theta)` values. \(\rho\) is measured in pixels and \(\theta\) is measured in radians. First parameter, Input image should be a binary image, so apply threshold or use canny edge detection before applying hough transform. Second and third parameters are \(\rho\) and \(\theta\) accuracies respectively. Fourth argument is the threshold, which means the minimum vote it should get to be considered as a line. Remember, number of votes depends upon the number of points on the line. So it represents the minimum length of line that should be detected. 

import cv2 as cvimport numpy as npimg = [cv.imread](../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56 "../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56")([cv.samples.findFile](../../d6/dba/group__core__utils__samples.html#ga3a33b00033b46c698ff6340d95569c13 "../../d6/dba/group__core__utils__samples.html#ga3a33b00033b46c698ff6340d95569c13")('sudoku.png'))gray = [cv.cvtColor](../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab "../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab")(img,cv.COLOR\_BGR2GRAY)edges = [cv.Canny](../../dd/d1a/group__imgproc__feature.html#ga2a671611e104c093843d7b7fc46d24af "../../dd/d1a/group__imgproc__feature.html#ga2a671611e104c093843d7b7fc46d24af")(gray,50,150,apertureSize = 3)lines = [cv.HoughLines](../../dd/d1a/group__imgproc__feature.html#ga46b4e588934f6c8dfd509cc6e0e4545a "../../dd/d1a/group__imgproc__feature.html#ga46b4e588934f6c8dfd509cc6e0e4545a")(edges,1,np.pi/180,200)for line in lines: rho,theta = line[0] a = np.cos(theta) b = np.sin(theta) x0 = a\*rho y0 = b\*rho x1 = int(x0 + 1000\*(-b)) y1 = int(y0 + 1000\*(a)) x2 = int(x0 - 1000\*(-b)) y2 = int(y0 - 1000\*(a)) [cv.line](../../d6/d6e/group__imgproc__draw.html#ga7078a9fae8c7e7d13d24dac2520ae4a2 "../../d6/d6e/group__imgproc__draw.html#ga7078a9fae8c7e7d13d24dac2520ae4a2")(img,(x1,y1),(x2,y2),(0,0,255),2)[cv.imwrite](../../d4/da8/group__imgcodecs.html#gabbc7ef1aa2edfaa87772f1202d67e0ce "../../d4/da8/group__imgcodecs.html#gabbc7ef1aa2edfaa87772f1202d67e0ce")('houghlines3.jpg',img) Check the results below:

![houghlines3.jpg](../../houghlines3.jpg)

image
## Probabilistic Hough Transform

In the hough transform, you can see that even for a line with two arguments, it takes a lot of computation. Probabilistic Hough Transform is an optimization of the Hough Transform we saw. It doesn't take all the points into consideration. Instead, it takes only a random subset of points which is sufficient for line detection. We just have to decrease the threshold. See image below which compares Hough Transform and Probabilistic Hough Transform in Hough space. (Image Courtesy : [Franck Bettinger's home page](http://phdfb1.free.fr/robot/mscthesis/node14.html "http://phdfb1.free.fr/robot/mscthesis/node14.html") )

![houghlines4.png](../../houghlines4.png)

image
 OpenCV implementation is based on Robust Detection of Lines Using the Progressive Probabilistic Hough Transform by Matas, J. and Galambos, C. and Kittler, J.V. [[172]](../../d0/de3/citelist.html#CITEREF_Matas00 "../../d0/de3/citelist.html#CITEREF_Matas00"). The function used is **[cv.HoughLinesP()](../../dd/d1a/group__imgproc__feature.html#ga8618180a5948286384e3b7ca02f6feeb "Finds line segments in a binary image using the probabilistic Hough transform. ")**. It has two new arguments.

* **minLineLength** - Minimum length of line. Line segments shorter than this are rejected.
* **maxLineGap** - Maximum allowed gap between line segments to treat them as a single line.

Best thing is that, it directly returns the two endpoints of lines. In previous case, you got only the parameters of lines, and you had to find all the points. Here, everything is direct and simple. 

import cv2 as cvimport numpy as npimg = [cv.imread](../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56 "../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56")([cv.samples.findFile](../../d6/dba/group__core__utils__samples.html#ga3a33b00033b46c698ff6340d95569c13 "../../d6/dba/group__core__utils__samples.html#ga3a33b00033b46c698ff6340d95569c13")('sudoku.png'))gray = [cv.cvtColor](../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab "../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab")(img,cv.COLOR\_BGR2GRAY)edges = [cv.Canny](../../dd/d1a/group__imgproc__feature.html#ga2a671611e104c093843d7b7fc46d24af "../../dd/d1a/group__imgproc__feature.html#ga2a671611e104c093843d7b7fc46d24af")(gray,50,150,apertureSize = 3)lines = [cv.HoughLinesP](../../dd/d1a/group__imgproc__feature.html#ga8618180a5948286384e3b7ca02f6feeb "../../dd/d1a/group__imgproc__feature.html#ga8618180a5948286384e3b7ca02f6feeb")(edges,1,np.pi/180,100,minLineLength=100,maxLineGap=10)for line in lines: x1,y1,x2,y2 = line[0] [cv.line](../../d6/d6e/group__imgproc__draw.html#ga7078a9fae8c7e7d13d24dac2520ae4a2 "../../d6/d6e/group__imgproc__draw.html#ga7078a9fae8c7e7d13d24dac2520ae4a2")(img,(x1,y1),(x2,y2),(0,255,0),2)[cv.imwrite](../../d4/da8/group__imgcodecs.html#gabbc7ef1aa2edfaa87772f1202d67e0ce "../../d4/da8/group__imgcodecs.html#gabbc7ef1aa2edfaa87772f1202d67e0ce")('houghlines5.jpg',img) See the results below:

![houghlines5.jpg](../../houghlines5.jpg)

image
## Additional Resources

1. [Hough Transform on Wikipedia](https://en.wikipedia.org/wiki/Hough_transform "https://en.wikipedia.org/wiki/Hough_transform")

## Exercises

---

Generated on Wed Oct 4 2023 23:37:11 for OpenCV by  [![doxygen](../../doxygen.png)](http://www.doxygen.org/index.html "http://www.doxygen.org/index.html") 1.8.13

//<![CDATA[
addTutorialsButtons();
//]]>

