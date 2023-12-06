

OpenCV: Harris Corner Detection

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
* [Feature Detection and Description](../../db/d27/tutorial_py_table_of_contents_feature2d.html "../../db/d27/tutorial_py_table_of_contents_feature2d.html")

Harris Corner Detection  

## Goal

In this chapter,

* We will understand the concepts behind Harris Corner Detection.
* We will see the following functions: **[cv.cornerHarris()](../../dd/d1a/group__imgproc__feature.html#gac1fc3598018010880e370e2f709b4345 "Harris corner detector. ")**, **[cv.cornerSubPix()](../../dd/d1a/group__imgproc__feature.html#ga354e0d7c86d0d9da75de9b9701a9a87e "Refines the corner locations. ")**

## Theory

In the last chapter, we saw that corners are regions in the image with large variation in intensity in all the directions. One early attempt to find these corners was done by **Chris Harris & Mike Stephens** in their paper **A Combined Corner and Edge Detector** in 1988, so now it is called the Harris Corner Detector. He took this simple idea to a mathematical form. It basically finds the difference in intensity for a displacement of \((u,v)\) in all directions. This is expressed as below:

\[E(u,v) = \sum\_{x,y} \underbrace{w(x,y)}\_\text{window function} \, [\underbrace{I(x+u,y+v)}\_\text{shifted intensity}-\underbrace{I(x,y)}\_\text{intensity}]^2\]

The window function is either a rectangular window or a Gaussian window which gives weights to pixels underneath.

We have to maximize this function \(E(u,v)\) for corner detection. That means we have to maximize the second term. Applying Taylor Expansion to the above equation and using some mathematical steps (please refer to any standard text books you like for full derivation), we get the final equation as:

\[E(u,v) \approx \begin{bmatrix} u & v \end{bmatrix} M \begin{bmatrix} u \\ v \end{bmatrix}\]

where

\[M = \sum\_{x,y} w(x,y) \begin{bmatrix}I\_x I\_x & I\_x I\_y \\ I\_x I\_y & I\_y I\_y \end{bmatrix}\]

Here, \(I\_x\) and \(I\_y\) are image derivatives in x and y directions respectively. (These can be easily found using **[cv.Sobel()](../../d4/d86/group__imgproc__filter.html#gacea54f142e81b6758cb6f375ce782c8d "Calculates the first, second, third, or mixed image derivatives using an extended Sobel operator...")**).

Then comes the main part. After this, they created a score, basically an equation, which determines if a window can contain a corner or not.

\[R = \det(M) - k(\operatorname{trace}(M))^2\]

where

* \(\det(M) = \lambda\_1 \lambda\_2\)
* \(\operatorname{trace}(M) = \lambda\_1 + \lambda\_2\)
* \(\lambda\_1\) and \(\lambda\_2\) are the eigenvalues of \(M\)

So the magnitudes of these eigenvalues decide whether a region is a corner, an edge, or flat.

* When \(|R|\) is small, which happens when \(\lambda\_1\) and \(\lambda\_2\) are small, the region is flat.
* When \(R<0\), which happens when \(\lambda\_1 >> \lambda\_2\) or vice versa, the region is edge.
* When \(R\) is large, which happens when \(\lambda\_1\) and \(\lambda\_2\) are large and \(\lambda\_1 \sim \lambda\_2\), the region is a corner.

It can be represented in a nice picture as follows:

![harris_region.jpg](../../harris_region.jpg)

image
 So the result of Harris Corner Detection is a grayscale image with these scores. Thresholding for a suitable score gives you the corners in the image. We will do it with a simple image.

## Harris Corner Detector in OpenCV

OpenCV has the function **[cv.cornerHarris()](../../dd/d1a/group__imgproc__feature.html#gac1fc3598018010880e370e2f709b4345 "Harris corner detector. ")** for this purpose. Its arguments are:

* **img** - Input image. It should be grayscale and float32 type.
* **blockSize** - It is the size of neighbourhood considered for corner detection
* **ksize** - Aperture parameter of the Sobel derivative used.
* **k** - Harris detector free parameter in the equation.

See the example below: 

import numpy as npimport cv2 as cvfilename = 'chessboard.png'img = [cv.imread](../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56 "../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56")(filename)gray = [cv.cvtColor](../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab "../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab")(img,cv.COLOR\_BGR2GRAY)gray = np.float32(gray)dst = [cv.cornerHarris](../../dd/d1a/group__imgproc__feature.html#gac1fc3598018010880e370e2f709b4345 "../../dd/d1a/group__imgproc__feature.html#gac1fc3598018010880e370e2f709b4345")(gray,2,3,0.04)#result is dilated for marking the corners, not importantdst = [cv.dilate](../../d4/d86/group__imgproc__filter.html#ga4ff0f3318642c4f469d0e11f242f3b6c "../../d4/d86/group__imgproc__filter.html#ga4ff0f3318642c4f469d0e11f242f3b6c")(dst,None)# Threshold for an optimal value, it may vary depending on the image.img[dst>0.01\*dst.max()]=[0,0,255][cv.imshow](../../df/d24/group__highgui__opengl.html#gaae7e90aa3415c68dba22a5ff2cefc25d "../../df/d24/group__highgui__opengl.html#gaae7e90aa3415c68dba22a5ff2cefc25d")('dst',img)if [cv.waitKey](../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7 "../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7")(0) & 0xff == 27: [cv.destroyAllWindows](../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481 "../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481")() Below are the three results:

![harris_result.jpg](../../harris_result.jpg)

image
## Corner with SubPixel Accuracy

Sometimes, you may need to find the corners with maximum accuracy. OpenCV comes with a function **[cv.cornerSubPix()](../../dd/d1a/group__imgproc__feature.html#ga354e0d7c86d0d9da75de9b9701a9a87e "Refines the corner locations. ")** which further refines the corners detected with sub-pixel accuracy. Below is an example. As usual, we need to find the Harris corners first. Then we pass the centroids of these corners (There may be a bunch of pixels at a corner, we take their centroid) to refine them. Harris corners are marked in red pixels and refined corners are marked in green pixels. For this function, we have to define the criteria when to stop the iteration. We stop it after a specified number of iterations or a certain accuracy is achieved, whichever occurs first. We also need to define the size of the neighbourhood it searches for corners. 

import numpy as npimport cv2 as cvfilename = 'chessboard2.jpg'img = [cv.imread](../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56 "../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56")(filename)gray = [cv.cvtColor](../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab "../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab")(img,cv.COLOR\_BGR2GRAY)# find Harris cornersgray = np.float32(gray)dst = [cv.cornerHarris](../../dd/d1a/group__imgproc__feature.html#gac1fc3598018010880e370e2f709b4345 "../../dd/d1a/group__imgproc__feature.html#gac1fc3598018010880e370e2f709b4345")(gray,2,3,0.04)dst = [cv.dilate](../../d4/d86/group__imgproc__filter.html#ga4ff0f3318642c4f469d0e11f242f3b6c "../../d4/d86/group__imgproc__filter.html#ga4ff0f3318642c4f469d0e11f242f3b6c")(dst,None)ret, dst = [cv.threshold](../../d7/d1b/group__imgproc__misc.html#gae8a4a146d1ca78c626a53577199e9c57 "../../d7/d1b/group__imgproc__misc.html#gae8a4a146d1ca78c626a53577199e9c57")(dst,0.01\*dst.max(),255,0)dst = np.uint8(dst)# find centroidsret, labels, stats, centroids = [cv.connectedComponentsWithStats](../../d3/dc0/group__imgproc__shape.html#gae57b028a2b2ca327227c2399a9d53241 "../../d3/dc0/group__imgproc__shape.html#gae57b028a2b2ca327227c2399a9d53241")(dst)# define the criteria to stop and refine the cornerscriteria = (cv.TERM\_CRITERIA\_EPS + cv.TERM\_CRITERIA\_MAX\_ITER, 100, 0.001)corners = [cv.cornerSubPix](../../dd/d1a/group__imgproc__feature.html#ga354e0d7c86d0d9da75de9b9701a9a87e "../../dd/d1a/group__imgproc__feature.html#ga354e0d7c86d0d9da75de9b9701a9a87e")(gray,np.float32(centroids),(5,5),(-1,-1),criteria)# Now draw themres = np.hstack((centroids,corners))res = np.int0(res)img[res[:,1],res[:,0]]=[0,0,255]img[res[:,3],res[:,2]] = [0,255,0][cv.imwrite](../../d4/da8/group__imgcodecs.html#gabbc7ef1aa2edfaa87772f1202d67e0ce "../../d4/da8/group__imgcodecs.html#gabbc7ef1aa2edfaa87772f1202d67e0ce")('subpixel5.png',img) Below is the result, where some important locations are shown in the zoomed window to visualize:

![subpixel3.png](../../subpixel3.png)

image
## Additional Resources

## Exercises

---

Generated on Wed Oct 4 2023 23:37:11 for OpenCV by  [![doxygen](../../doxygen.png)](http://www.doxygen.org/index.html "http://www.doxygen.org/index.html") 1.8.13

//<![CDATA[
addTutorialsButtons();
//]]>

