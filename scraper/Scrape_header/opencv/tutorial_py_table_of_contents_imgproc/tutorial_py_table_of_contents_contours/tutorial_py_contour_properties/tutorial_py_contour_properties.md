

OpenCV: Contour Properties

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

Contour Properties  

**Prev Tutorial:** [Contour Features](../../dd/d49/tutorial_py_contour_features.html "../../dd/d49/tutorial_py_contour_features.html")

**Next Tutorial:** [Contours : More Functions](../../d5/d45/tutorial_py_contours_more_functions.html "../../d5/d45/tutorial_py_contours_more_functions.html")

Here we will learn to extract some frequently used properties of objects like Solidity, Equivalent Diameter, Mask image, Mean Intensity etc. More features can be found at [Matlab regionprops documentation](http://www.mathworks.in/help/images/ref/regionprops.html "http://www.mathworks.in/help/images/ref/regionprops.html").

\*(NB : Centroid, Area, Perimeter etc also belong to this category, but we have seen it in last chapter)\*

## 1. Aspect Ratio

It is the ratio of width to height of bounding rect of the object.

\[Aspect \; Ratio = \frac{Width}{Height}\]

x,y,w,h = [cv.boundingRect](../../d3/dc0/group__imgproc__shape.html#ga103fcbda2f540f3ef1c042d6a9b35ac7 "../../d3/dc0/group__imgproc__shape.html#ga103fcbda2f540f3ef1c042d6a9b35ac7")(cnt)aspect\_ratio = float(w)/h## 2. Extent

Extent is the ratio of contour area to bounding rectangle area.

\[Extent = \frac{Object \; Area}{Bounding \; Rectangle \; Area}\]

area = [cv.contourArea](../../d3/dc0/group__imgproc__shape.html#ga2c759ed9f497d4a618048a2f56dc97f1 "../../d3/dc0/group__imgproc__shape.html#ga2c759ed9f497d4a618048a2f56dc97f1")(cnt)x,y,w,h = [cv.boundingRect](../../d3/dc0/group__imgproc__shape.html#ga103fcbda2f540f3ef1c042d6a9b35ac7 "../../d3/dc0/group__imgproc__shape.html#ga103fcbda2f540f3ef1c042d6a9b35ac7")(cnt)rect\_area = w\*hextent = float(area)/rect\_area## 3. Solidity

Solidity is the ratio of contour area to its convex hull area.

\[Solidity = \frac{Contour \; Area}{Convex \; Hull \; Area}\]

area = [cv.contourArea](../../d3/dc0/group__imgproc__shape.html#ga2c759ed9f497d4a618048a2f56dc97f1 "../../d3/dc0/group__imgproc__shape.html#ga2c759ed9f497d4a618048a2f56dc97f1")(cnt)hull = [cv.convexHull](../../d3/dc0/group__imgproc__shape.html#ga014b28e56cb8854c0de4a211cb2be656 "../../d3/dc0/group__imgproc__shape.html#ga014b28e56cb8854c0de4a211cb2be656")(cnt)hull\_area = [cv.contourArea](../../d3/dc0/group__imgproc__shape.html#ga2c759ed9f497d4a618048a2f56dc97f1 "../../d3/dc0/group__imgproc__shape.html#ga2c759ed9f497d4a618048a2f56dc97f1")(hull)solidity = float(area)/hull\_area## 4. Equivalent Diameter

Equivalent Diameter is the diameter of the circle whose area is same as the contour area.

\[Equivalent \; Diameter = \sqrt{\frac{4 \times Contour \; Area}{\pi}}\]

area = [cv.contourArea](../../d3/dc0/group__imgproc__shape.html#ga2c759ed9f497d4a618048a2f56dc97f1 "../../d3/dc0/group__imgproc__shape.html#ga2c759ed9f497d4a618048a2f56dc97f1")(cnt)equi\_diameter = np.sqrt(4\*area/np.pi)## 5. Orientation

Orientation is the angle at which object is directed. Following method also gives the Major Axis and Minor Axis lengths. 

(x,y),(MA,ma),angle = [cv.fitEllipse](../../d3/dc0/group__imgproc__shape.html#gaf259efaad93098103d6c27b9e4900ffa "../../d3/dc0/group__imgproc__shape.html#gaf259efaad93098103d6c27b9e4900ffa")(cnt)## 6. Mask and Pixel Points

In some cases, we may need all the points which comprises that object. It can be done as follows: 

mask = np.zeros(imgray.shape,np.uint8)[cv.drawContours](../../d6/d6e/group__imgproc__draw.html#ga746c0625f1781f1ffc9056259103edbc "../../d6/d6e/group__imgproc__draw.html#ga746c0625f1781f1ffc9056259103edbc")(mask,[cnt],0,255,-1)pixelpoints = np.transpose(np.nonzero(mask))#pixelpoints = cv.findNonZero(mask) Here, two methods, one using Numpy functions, next one using OpenCV function (last commented line) are given to do the same. Results are also same, but with a slight difference. Numpy gives coordinates in \*\*(row, column)\*\* format, while OpenCV gives coordinates in \*\*(x,y)\*\* format. So basically the answers will be interchanged. Note that, **row = y** and **column = x**.

## 7. Maximum Value, Minimum Value and their locations

We can find these parameters using a mask image. 

min\_val, max\_val, min\_loc, max\_loc = [cv.minMaxLoc](../../d2/de8/group__core__array.html#ga8873b86a29c5af51cafdcee82f8150a7 "../../d2/de8/group__core__array.html#ga8873b86a29c5af51cafdcee82f8150a7")(imgray,mask = mask)## 8. Mean Color or Mean Intensity

Here, we can find the average color of an object. Or it can be average intensity of the object in grayscale mode. We again use the same mask to do it. 

mean\_val = [cv.mean](../../d2/de8/group__core__array.html#ga191389f8a0e58180bb13a727782cd461 "../../d2/de8/group__core__array.html#ga191389f8a0e58180bb13a727782cd461")(im,mask = mask)## 9. Extreme Points

Extreme Points means topmost, bottommost, rightmost and leftmost points of the object. 

leftmost = tuple(cnt[cnt[:,:,0].argmin()][0])rightmost = tuple(cnt[cnt[:,:,0].argmax()][0])topmost = tuple(cnt[cnt[:,:,1].argmin()][0])bottommost = tuple(cnt[cnt[:,:,1].argmax()][0]) For eg, if I apply it to an Indian map, I get the following result :

![extremepoints.jpg](../../extremepoints.jpg)

image
## Additional Resources

## Exercises

1. There are still some features left in matlab regionprops doc. Try to implement them.

---

Generated on Wed Oct 4 2023 23:37:11 for OpenCV by  [![doxygen](../../doxygen.png)](http://www.doxygen.org/index.html "http://www.doxygen.org/index.html") 1.8.13

//<![CDATA[
addTutorialsButtons();
//]]>

