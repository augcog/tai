

OpenCV: Geometric Transformations of Images

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

Geometric Transformations of Images  

## Goals

* Learn to apply different geometric transformations to images, like translation, rotation, affine transformation etc.
* You will see these functions: **[cv.getPerspectiveTransform](../../da/d54/group__imgproc__transform.html#ga20f62aa3235d869c9956436c870893ae "Calculates a perspective transform from four pairs of the corresponding points. ")**

## Transformations

OpenCV provides two transformation functions, **[cv.warpAffine](../../da/d54/group__imgproc__transform.html#ga0203d9ee5fcd28d40dbc4a1ea4451983 "Applies an affine transformation to an image. ")** and **[cv.warpPerspective](../../da/d54/group__imgproc__transform.html#gaf73673a7e8e18ec6963e3774e6a94b87 "Applies a perspective transformation to an image. ")**, with which you can perform all kinds of transformations. **[cv.warpAffine](../../da/d54/group__imgproc__transform.html#ga0203d9ee5fcd28d40dbc4a1ea4451983 "Applies an affine transformation to an image. ")** takes a 2x3 transformation matrix while **[cv.warpPerspective](../../da/d54/group__imgproc__transform.html#gaf73673a7e8e18ec6963e3774e6a94b87 "Applies a perspective transformation to an image. ")** takes a 3x3 transformation matrix as input.

### Scaling

Scaling is just resizing of the image. OpenCV comes with a function **[cv.resize()](../../da/d54/group__imgproc__transform.html#ga47a974309e9102f5f08231edc7e7529d "Resizes an image. ")** for this purpose. The size of the image can be specified manually, or you can specify the scaling factor. Different interpolation methods are used. Preferable interpolation methods are **[cv.INTER\_AREA](../../da/d54/group__imgproc__transform.html#gga5bb5a1fea74ea38e1a5445ca803ff121acf959dca2480cc694ca016b81b442ceb "../../da/d54/group__imgproc__transform.html#gga5bb5a1fea74ea38e1a5445ca803ff121acf959dca2480cc694ca016b81b442ceb")** for shrinking and **[cv.INTER\_CUBIC](../../da/d54/group__imgproc__transform.html#gga5bb5a1fea74ea38e1a5445ca803ff121a55e404e7fa9684af79fe9827f36a5dc1 "../../da/d54/group__imgproc__transform.html#gga5bb5a1fea74ea38e1a5445ca803ff121a55e404e7fa9684af79fe9827f36a5dc1")** (slow) & **[cv.INTER\_LINEAR](../../da/d54/group__imgproc__transform.html#gga5bb5a1fea74ea38e1a5445ca803ff121ac97d8e4880d8b5d509e96825c7522deb "../../da/d54/group__imgproc__transform.html#gga5bb5a1fea74ea38e1a5445ca803ff121ac97d8e4880d8b5d509e96825c7522deb")** for zooming. By default, the interpolation method **[cv.INTER\_LINEAR](../../da/d54/group__imgproc__transform.html#gga5bb5a1fea74ea38e1a5445ca803ff121ac97d8e4880d8b5d509e96825c7522deb "../../da/d54/group__imgproc__transform.html#gga5bb5a1fea74ea38e1a5445ca803ff121ac97d8e4880d8b5d509e96825c7522deb")** is used for all resizing purposes. You can resize an input image with either of following methods: 

import numpy as npimport cv2 as cvimg = [cv.imread](../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56 "../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56")('messi5.jpg')assert img is not None, "file could not be read, check with os.path.exists()"res = [cv.resize](../../da/d54/group__imgproc__transform.html#ga47a974309e9102f5f08231edc7e7529d "../../da/d54/group__imgproc__transform.html#ga47a974309e9102f5f08231edc7e7529d")(img,None,fx=2, fy=2, interpolation = cv.INTER\_CUBIC)#ORheight, width = img.shape[:2]res = [cv.resize](../../da/d54/group__imgproc__transform.html#ga47a974309e9102f5f08231edc7e7529d "../../da/d54/group__imgproc__transform.html#ga47a974309e9102f5f08231edc7e7529d")(img,(2\*width, 2\*height), interpolation = cv.INTER\_CUBIC) ### Translation

Translation is the shifting of an object's location. If you know the shift in the (x,y) direction and let it be \((t\_x,t\_y)\), you can create the transformation matrix \(\textbf{M}\) as follows:

\[M = \begin{bmatrix} 1 & 0 & t\_x \\ 0 & 1 & t\_y \end{bmatrix}\]

You can take make it into a Numpy array of type np.float32 and pass it into the **[cv.warpAffine()](../../da/d54/group__imgproc__transform.html#ga0203d9ee5fcd28d40dbc4a1ea4451983 "Applies an affine transformation to an image. ")** function. See the below example for a shift of (100,50): 

import numpy as npimport cv2 as cvimg = [cv.imread](../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56 "../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56")('messi5.jpg', cv.IMREAD\_GRAYSCALE)assert img is not None, "file could not be read, check with os.path.exists()"rows,cols = img.shapeM = np.float32([[1,0,100],[0,1,50]])dst = [cv.warpAffine](../../da/d54/group__imgproc__transform.html#ga0203d9ee5fcd28d40dbc4a1ea4451983 "../../da/d54/group__imgproc__transform.html#ga0203d9ee5fcd28d40dbc4a1ea4451983")(img,M,(cols,rows))[cv.imshow](../../df/d24/group__highgui__opengl.html#gaae7e90aa3415c68dba22a5ff2cefc25d "../../df/d24/group__highgui__opengl.html#gaae7e90aa3415c68dba22a5ff2cefc25d")('img',dst)[cv.waitKey](../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7 "../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7")(0)[cv.destroyAllWindows](../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481 "../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481")() **warning**

The third argument of the **[cv.warpAffine()](../../da/d54/group__imgproc__transform.html#ga0203d9ee5fcd28d40dbc4a1ea4451983 "Applies an affine transformation to an image. ")** function is the size of the output image, which should be in the form of \*\*(width, height)\*\*. Remember width = number of columns, and height = number of rows.

See the result below:

![translation.jpg](../../translation.jpg)

image
### Rotation

Rotation of an image for an angle \(\theta\) is achieved by the transformation matrix of the form

\[M = \begin{bmatrix} cos\theta & -sin\theta \\ sin\theta & cos\theta \end{bmatrix}\]

But OpenCV provides scaled rotation with adjustable center of rotation so that you can rotate at any location you prefer. The modified transformation matrix is given by

\[\begin{bmatrix} \alpha & \beta & (1- \alpha ) \cdot center.x - \beta \cdot center.y \\ - \beta & \alpha & \beta \cdot center.x + (1- \alpha ) \cdot center.y \end{bmatrix}\]

where:

\[\begin{array}{l} \alpha = scale \cdot \cos \theta , \\ \beta = scale \cdot \sin \theta \end{array}\]

To find this transformation matrix, OpenCV provides a function, **[cv.getRotationMatrix2D](../../da/d54/group__imgproc__transform.html#gafbbc470ce83812914a70abfb604f4326 "Calculates an affine matrix of 2D rotation. ")**. Check out the below example which rotates the image by 90 degree with respect to center without any scaling. 

img = [cv.imread](../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56 "../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56")('messi5.jpg', cv.IMREAD\_GRAYSCALE)assert img is not None, "file could not be read, check with os.path.exists()"rows,cols = img.shape# cols-1 and rows-1 are the coordinate limits.M = [cv.getRotationMatrix2D](../../da/d54/group__imgproc__transform.html#gafbbc470ce83812914a70abfb604f4326 "../../da/d54/group__imgproc__transform.html#gafbbc470ce83812914a70abfb604f4326")(((cols-1)/2.0,(rows-1)/2.0),90,1)dst = [cv.warpAffine](../../da/d54/group__imgproc__transform.html#ga0203d9ee5fcd28d40dbc4a1ea4451983 "../../da/d54/group__imgproc__transform.html#ga0203d9ee5fcd28d40dbc4a1ea4451983")(img,M,(cols,rows)) See the result:

![rotation.jpg](../../rotation.jpg)

image
### Affine Transformation

In affine transformation, all parallel lines in the original image will still be parallel in the output image. To find the transformation matrix, we need three points from the input image and their corresponding locations in the output image. Then **[cv.getAffineTransform](../../da/d54/group__imgproc__transform.html#ga8f6d378f9f8eebb5cb55cd3ae295a999 "Calculates an affine transform from three pairs of the corresponding points. ")** will create a 2x3 matrix which is to be passed to **[cv.warpAffine](../../da/d54/group__imgproc__transform.html#ga0203d9ee5fcd28d40dbc4a1ea4451983 "Applies an affine transformation to an image. ")**.

Check the below example, and also look at the points I selected (which are marked in green color): 

img = [cv.imread](../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56 "../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56")('drawing.png')assert img is not None, "file could not be read, check with os.path.exists()"rows,cols,ch = img.shapepts1 = np.float32([[50,50],[200,50],[50,200]])pts2 = np.float32([[10,100],[200,50],[100,250]])M = [cv.getAffineTransform](../../da/d54/group__imgproc__transform.html#ga47069038267385913c61334e3d6af2e0 "../../da/d54/group__imgproc__transform.html#ga47069038267385913c61334e3d6af2e0")(pts1,pts2)dst = [cv.warpAffine](../../da/d54/group__imgproc__transform.html#ga0203d9ee5fcd28d40dbc4a1ea4451983 "../../da/d54/group__imgproc__transform.html#ga0203d9ee5fcd28d40dbc4a1ea4451983")(img,M,(cols,rows))plt.subplot(121),plt.imshow(img),plt.title('Input')plt.subplot(122),plt.imshow(dst),plt.title('Output')plt.show() See the result:

![affine.jpg](../../affine.jpg)

image
### Perspective Transformation

For perspective transformation, you need a 3x3 transformation matrix. Straight lines will remain straight even after the transformation. To find this transformation matrix, you need 4 points on the input image and corresponding points on the output image. Among these 4 points, 3 of them should not be collinear. Then the transformation matrix can be found by the function **[cv.getPerspectiveTransform](../../da/d54/group__imgproc__transform.html#ga20f62aa3235d869c9956436c870893ae "Calculates a perspective transform from four pairs of the corresponding points. ")**. Then apply **[cv.warpPerspective](../../da/d54/group__imgproc__transform.html#gaf73673a7e8e18ec6963e3774e6a94b87 "Applies a perspective transformation to an image. ")** with this 3x3 transformation matrix.

See the code below: 

img = [cv.imread](../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56 "../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56")('sudoku.png')assert img is not None, "file could not be read, check with os.path.exists()"rows,cols,ch = img.shapepts1 = np.float32([[56,65],[368,52],[28,387],[389,390]])pts2 = np.float32([[0,0],[300,0],[0,300],[300,300]])M = [cv.getPerspectiveTransform](../../da/d54/group__imgproc__transform.html#gae66ba39ba2e47dd0750555c7e986ab85 "../../da/d54/group__imgproc__transform.html#gae66ba39ba2e47dd0750555c7e986ab85")(pts1,pts2)dst = [cv.warpPerspective](../../da/d54/group__imgproc__transform.html#gaf73673a7e8e18ec6963e3774e6a94b87 "../../da/d54/group__imgproc__transform.html#gaf73673a7e8e18ec6963e3774e6a94b87")(img,M,(300,300))plt.subplot(121),plt.imshow(img),plt.title('Input')plt.subplot(122),plt.imshow(dst),plt.title('Output')plt.show() Result:

![perspective.jpg](../../perspective.jpg)

image
## Additional Resources

1. "Computer Vision: Algorithms and Applications", Richard Szeliski

## Exercises

---

Generated on Wed Oct 4 2023 23:37:11 for OpenCV by  [![doxygen](../../doxygen.png)](http://www.doxygen.org/index.html "http://www.doxygen.org/index.html") 1.8.13

//<![CDATA[
addTutorialsButtons();
//]]>

