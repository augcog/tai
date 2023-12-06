

OpenCV: Contour Features

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

Contour Features  

**Prev Tutorial:** [Contours : Getting Started](../../d4/d73/tutorial_py_contours_begin.html "../../d4/d73/tutorial_py_contours_begin.html")

**Next Tutorial:** [Contour Properties](../../d1/d32/tutorial_py_contour_properties.html "../../d1/d32/tutorial_py_contour_properties.html")

## Goal

In this article, we will learn

* To find the different features of contours, like area, perimeter, centroid, bounding box etc
* You will see plenty of functions related to contours.

## 1. Moments

Image moments help you to calculate some features like center of mass of the object, area of the object etc. Check out the wikipedia page on [Image Moments](https://en.wikipedia.org/wiki/Image_moment "https://en.wikipedia.org/wiki/Image_moment")

The function **[cv.moments()](../../d3/dc0/group__imgproc__shape.html#ga556a180f43cab22649c23ada36a8a139 "Calculates all of the moments up to the third order of a polygon or rasterized shape. ")** gives a dictionary of all moment values calculated. See below: 

import numpy as npimport cv2 as cvimg = [cv.imread](../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56 "../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56")('star.jpg', cv.IMREAD\_GRAYSCALE)assert img is not None, "file could not be read, check with os.path.exists()"ret,thresh = [cv.threshold](../../d7/d1b/group__imgproc__misc.html#gae8a4a146d1ca78c626a53577199e9c57 "../../d7/d1b/group__imgproc__misc.html#gae8a4a146d1ca78c626a53577199e9c57")(img,127,255,0)contours,hierarchy = [cv.findContours](../../d3/dc0/group__imgproc__shape.html#gae4156f04053c44f886e387cff0ef6e08 "../../d3/dc0/group__imgproc__shape.html#gae4156f04053c44f886e387cff0ef6e08")(thresh, 1, 2)cnt = contours[0]M = [cv.moments](../../d3/dc0/group__imgproc__shape.html#ga556a180f43cab22649c23ada36a8a139 "../../d3/dc0/group__imgproc__shape.html#ga556a180f43cab22649c23ada36a8a139")(cnt)[print](../../df/d57/namespacecv_1_1dnn.html#a43417dcaeb3c1e2a09b9d948e234c366 "../../df/d57/namespacecv_1_1dnn.html#a43417dcaeb3c1e2a09b9d948e234c366")( M ) From this moments, you can extract useful data like area, centroid etc. Centroid is given by the relations, \(C\_x = \frac{M\_{10}}{M\_{00}}\) and \(C\_y = \frac{M\_{01}}{M\_{00}}\). This can be done as follows: 

cx = int(M['m10']/M['m00'])cy = int(M['m01']/M['m00'])## 2. Contour Area

Contour area is given by the function **[cv.contourArea()](../../d3/dc0/group__imgproc__shape.html#ga2c759ed9f497d4a618048a2f56dc97f1 "Calculates a contour area. ")** or from moments, **M['m00']**. 

area = [cv.contourArea](../../d3/dc0/group__imgproc__shape.html#ga2c759ed9f497d4a618048a2f56dc97f1 "../../d3/dc0/group__imgproc__shape.html#ga2c759ed9f497d4a618048a2f56dc97f1")(cnt)## 3. Contour Perimeter

It is also called arc length. It can be found out using **[cv.arcLength()](../../d3/dc0/group__imgproc__shape.html#ga8d26483c636be6b35c3ec6335798a47c "Calculates a contour perimeter or a curve length. ")** function. Second argument specify whether shape is a closed contour (if passed True), or just a curve. 

perimeter = [cv.arcLength](../../d3/dc0/group__imgproc__shape.html#ga8d26483c636be6b35c3ec6335798a47c "../../d3/dc0/group__imgproc__shape.html#ga8d26483c636be6b35c3ec6335798a47c")(cnt,True)## 4. Contour Approximation

It approximates a contour shape to another shape with less number of vertices depending upon the precision we specify. It is an implementation of [Douglas-Peucker algorithm](https://en.wikipedia.org/wiki/Ramer-Douglas-Peucker_algorithm "https://en.wikipedia.org/wiki/Ramer-Douglas-Peucker_algorithm"). Check the wikipedia page for algorithm and demonstration.

To understand this, suppose you are trying to find a square in an image, but due to some problems in the image, you didn't get a perfect square, but a "bad shape" (As shown in first image below). Now you can use this function to approximate the shape. In this, second argument is called epsilon, which is maximum distance from contour to approximated contour. It is an accuracy parameter. A wise selection of epsilon is needed to get the correct output. 

epsilon = 0.1\*[cv.arcLength](../../d3/dc0/group__imgproc__shape.html#ga8d26483c636be6b35c3ec6335798a47c "../../d3/dc0/group__imgproc__shape.html#ga8d26483c636be6b35c3ec6335798a47c")(cnt,True)approx = [cv.approxPolyDP](../../d3/dc0/group__imgproc__shape.html#ga0012a5fdaea70b8a9970165d98722b4c "../../d3/dc0/group__imgproc__shape.html#ga0012a5fdaea70b8a9970165d98722b4c")(cnt,epsilon,True) Below, in second image, green line shows the approximated curve for epsilon = 10% of arc length. Third image shows the same for epsilon = 1% of the arc length. Third argument specifies whether curve is closed or not.

![approx.jpg](../../approx.jpg)

image
## 5. Convex Hull

Convex Hull will look similar to contour approximation, but it is not (Both may provide same results in some cases). Here, **[cv.convexHull()](../../d3/dc0/group__imgproc__shape.html#ga014b28e56cb8854c0de4a211cb2be656 "Finds the convex hull of a point set. ")** function checks a curve for convexity defects and corrects it. Generally speaking, convex curves are the curves which are always bulged out, or at-least flat. And if it is bulged inside, it is called convexity defects. For example, check the below image of hand. Red line shows the convex hull of hand. The double-sided arrow marks shows the convexity defects, which are the local maximum deviations of hull from contours.

![convexitydefects.jpg](../../convexitydefects.jpg)

image
 There is a little bit things to discuss about it its syntax: 

hull = [cv.convexHull](../../d3/dc0/group__imgproc__shape.html#ga014b28e56cb8854c0de4a211cb2be656 "../../d3/dc0/group__imgproc__shape.html#ga014b28e56cb8854c0de4a211cb2be656")(points[, hull[, clockwise[, returnPoints]]]) Arguments details:

* **points** are the contours we pass into.
* **hull** is the output, normally we avoid it.
* **clockwise** : Orientation flag. If it is True, the output convex hull is oriented clockwise. Otherwise, it is oriented counter-clockwise.
* **returnPoints** : By default, True. Then it returns the coordinates of the hull points. If False, it returns the indices of contour points corresponding to the hull points.

So to get a convex hull as in above image, following is sufficient: 

hull = [cv.convexHull](../../d3/dc0/group__imgproc__shape.html#ga014b28e56cb8854c0de4a211cb2be656 "../../d3/dc0/group__imgproc__shape.html#ga014b28e56cb8854c0de4a211cb2be656")(cnt) But if you want to find convexity defects, you need to pass returnPoints = False. To understand it, we will take the rectangle image above. First I found its contour as cnt. Now I found its convex hull with returnPoints = True, I got following values: [[[234 202]], [[ 51 202]], [[ 51 79]], [[234 79]]] which are the four corner points of rectangle. Now if do the same with returnPoints = False, I get following result: [[129],[ 67],[ 0],[142]]. These are the indices of corresponding points in contours. For eg, check the first value: cnt[129] = [[234, 202]] which is same as first result (and so on for others).

You will see it again when we discuss about convexity defects.

## 6. Checking Convexity

There is a function to check if a curve is convex or not, **[cv.isContourConvex()](../../d3/dc0/group__imgproc__shape.html#ga8abf8010377b58cbc16db6734d92941b "Tests a contour convexity. ")**. It just return whether True or False. Not a big deal. 

k = [cv.isContourConvex](../../d3/dc0/group__imgproc__shape.html#ga8abf8010377b58cbc16db6734d92941b "../../d3/dc0/group__imgproc__shape.html#ga8abf8010377b58cbc16db6734d92941b")(cnt)## 7. Bounding Rectangle

There are two types of bounding rectangles.

### 7.a. Straight Bounding Rectangle

It is a straight rectangle, it doesn't consider the rotation of the object. So area of the bounding rectangle won't be minimum. It is found by the function **[cv.boundingRect()](../../d3/dc0/group__imgproc__shape.html#ga103fcbda2f540f3ef1c042d6a9b35ac7 "Calculates the up-right bounding rectangle of a point set or non-zero pixels of gray-scale image...")**.

Let (x,y) be the top-left coordinate of the rectangle and (w,h) be its width and height. 

x,y,w,h = [cv.boundingRect](../../d3/dc0/group__imgproc__shape.html#ga103fcbda2f540f3ef1c042d6a9b35ac7 "../../d3/dc0/group__imgproc__shape.html#ga103fcbda2f540f3ef1c042d6a9b35ac7")(cnt)[cv.rectangle](../../d6/d6e/group__imgproc__draw.html#gac865734d137287c0afb7682ff7b3db23 "../../d6/d6e/group__imgproc__draw.html#gac865734d137287c0afb7682ff7b3db23")(img,(x,y),(x+w,y+h),(0,255,0),2)### 7.b. Rotated Rectangle

Here, bounding rectangle is drawn with minimum area, so it considers the rotation also. The function used is **[cv.minAreaRect()](../../d3/dc0/group__imgproc__shape.html#ga3d476a3417130ae5154aea421ca7ead9 "Finds a rotated rectangle of the minimum area enclosing the input 2D point set. ")**. It returns a Box2D structure which contains following details - ( center (x,y), (width, height), angle of rotation ). But to draw this rectangle, we need 4 corners of the rectangle. It is obtained by the function **[cv.boxPoints()](../../d3/dc0/group__imgproc__shape.html#gaf78d467e024b4d7936cf9397185d2f5c "Finds the four vertices of a rotated rect. Useful to draw the rotated rectangle. ")** 

rect = [cv.minAreaRect](../../d3/dc0/group__imgproc__shape.html#ga3d476a3417130ae5154aea421ca7ead9 "../../d3/dc0/group__imgproc__shape.html#ga3d476a3417130ae5154aea421ca7ead9")(cnt)box = [cv.boxPoints](../../d3/dc0/group__imgproc__shape.html#gaf78d467e024b4d7936cf9397185d2f5c "../../d3/dc0/group__imgproc__shape.html#gaf78d467e024b4d7936cf9397185d2f5c")(rect)box = np.int0(box)[cv.drawContours](../../d6/d6e/group__imgproc__draw.html#ga746c0625f1781f1ffc9056259103edbc "../../d6/d6e/group__imgproc__draw.html#ga746c0625f1781f1ffc9056259103edbc")(img,[box],0,(0,0,255),2) Both the rectangles are shown in a single image. Green rectangle shows the normal bounding rect. Red rectangle is the rotated rect.

![boundingrect.png](../../boundingrect.png)

image
## 8. Minimum Enclosing Circle

Next we find the circumcircle of an object using the function **[cv.minEnclosingCircle()](../../d3/dc0/group__imgproc__shape.html#ga8ce13c24081bbc7151e9326f412190f1 "Finds a circle of the minimum area enclosing a 2D point set. ")**. It is a circle which completely covers the object with minimum area. 

(x,y),radius = [cv.minEnclosingCircle](../../d3/dc0/group__imgproc__shape.html#ga8ce13c24081bbc7151e9326f412190f1 "../../d3/dc0/group__imgproc__shape.html#ga8ce13c24081bbc7151e9326f412190f1")(cnt)center = (int(x),int(y))radius = int(radius)[cv.circle](../../d6/d6e/group__imgproc__draw.html#gaf10604b069374903dbd0f0488cb43670 "../../d6/d6e/group__imgproc__draw.html#gaf10604b069374903dbd0f0488cb43670")(img,center,radius,(0,255,0),2) 
![circumcircle.png](../../circumcircle.png)

image
## 9. Fitting an Ellipse

Next one is to fit an ellipse to an object. It returns the rotated rectangle in which the ellipse is inscribed. 

ellipse = [cv.fitEllipse](../../d3/dc0/group__imgproc__shape.html#gaf259efaad93098103d6c27b9e4900ffa "../../d3/dc0/group__imgproc__shape.html#gaf259efaad93098103d6c27b9e4900ffa")(cnt)[cv.ellipse](../../d6/d6e/group__imgproc__draw.html#ga57be400d8eff22fb946ae90c8e7441f9 "../../d6/d6e/group__imgproc__draw.html#ga57be400d8eff22fb946ae90c8e7441f9")(img,ellipse,(0,255,0),2) 
![fitellipse.png](../../fitellipse.png)

image
## 10. Fitting a Line

Similarly we can fit a line to a set of points. Below image contains a set of white points. We can approximate a straight line to it. 

rows,cols = img.shape[:2][vx,vy,x,y] = [cv.fitLine](../../d3/dc0/group__imgproc__shape.html#gaf849da1fdafa67ee84b1e9a23b93f91f "../../d3/dc0/group__imgproc__shape.html#gaf849da1fdafa67ee84b1e9a23b93f91f")(cnt, cv.DIST\_L2,0,0.01,0.01)lefty = int((-x\*vy/vx) + y)righty = int(((cols-x)\*vy/vx)+y)[cv.line](../../d6/d6e/group__imgproc__draw.html#ga7078a9fae8c7e7d13d24dac2520ae4a2 "../../d6/d6e/group__imgproc__draw.html#ga7078a9fae8c7e7d13d24dac2520ae4a2")(img,(cols-1,righty),(0,lefty),(0,255,0),2) 
![fitline.jpg](../../fitline.jpg)

image
## Additional Resources

## Exercises

---

Generated on Wed Oct 4 2023 23:37:11 for OpenCV by  [![doxygen](../../doxygen.png)](http://www.doxygen.org/index.html "http://www.doxygen.org/index.html") 1.8.13

//<![CDATA[
addTutorialsButtons();
//]]>

