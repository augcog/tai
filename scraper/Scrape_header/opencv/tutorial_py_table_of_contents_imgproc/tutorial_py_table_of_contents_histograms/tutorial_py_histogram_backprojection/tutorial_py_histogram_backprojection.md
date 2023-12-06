

OpenCV: Histogram - 4 : Histogram Backprojection

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

Histogram - 4 : Histogram Backprojection  

## Goal

In this chapter, we will learn about histogram backprojection.

## Theory

It was proposed by **Michael J. Swain , Dana H. Ballard** in their paper **Indexing via color histograms**.

**What is it actually in simple words?** It is used for image segmentation or finding objects of interest in an image. In simple words, it creates an image of the same size (but single channel) as that of our input image, where each pixel corresponds to the probability of that pixel belonging to our object. In more simpler words, the output image will have our object of interest in more white compared to remaining part. Well, that is an intuitive explanation. (I can't make it more simpler). Histogram Backprojection is used with camshift algorithm etc.

**How do we do it ?** We create a histogram of an image containing our object of interest (in our case, the ground, leaving player and other things). The object should fill the image as far as possible for better results. And a color histogram is preferred over grayscale histogram, because color of the object is a better way to define the object than its grayscale intensity. We then "back-project" this histogram over our test image where we need to find the object, ie in other words, we calculate the probability of every pixel belonging to the ground and show it. The resulting output on proper thresholding gives us the ground alone.

## Algorithm in Numpy

1. First we need to calculate the color histogram of both the object we need to find (let it be 'M') and the image where we are going to search (let it be 'I'). import numpy as npimport cv2 as cvfrom matplotlib import pyplot as plt#roi is the object or region of object we need to findroi = [cv.imread](../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56 "../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56")('rose\_red.png')assert roi is not None, "file could not be read, check with os.path.exists()"hsv = [cv.cvtColor](../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab "../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab")(roi,cv.COLOR\_BGR2HSV)#target is the image we search intarget = [cv.imread](../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56 "../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56")('rose.png')assert target is not None, "file could not be read, check with os.path.exists()"hsvt = [cv.cvtColor](../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab "../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab")(target,cv.COLOR\_BGR2HSV)# Find the histograms using calcHist. Can be done with np.histogram2d alsoM = [cv.calcHist](../../d6/dc7/group__imgproc__hist.html#ga6ca1876785483836f72a77ced8ea759a "../../d6/dc7/group__imgproc__hist.html#ga6ca1876785483836f72a77ced8ea759a")([hsv],[0, 1], None, [180, 256], [0, 180, 0, 256] )I = [cv.calcHist](../../d6/dc7/group__imgproc__hist.html#ga6ca1876785483836f72a77ced8ea759a "../../d6/dc7/group__imgproc__hist.html#ga6ca1876785483836f72a77ced8ea759a")([hsvt],[0, 1], None, [180, 256], [0, 180, 0, 256] )
2. Find the ratio \(R = \frac{M}{I}\). Then backproject R, ie use R as palette and create a new image with every pixel as its corresponding probability of being target. ie B(x,y) = R[h(x,y),s(x,y)] where h is hue and s is saturation of the pixel at (x,y). After that apply the condition \(B(x,y) = min[B(x,y), 1]\). h,s,v = [cv.split](../../d2/de8/group__core__array.html#ga8027f9deee1e42716be8039e5863fbd9 "../../d2/de8/group__core__array.html#ga8027f9deee1e42716be8039e5863fbd9")(hsvt)B = R[h.ravel(),s.ravel()]B = np.minimum(B,1)B = B.reshape(hsvt.shape[:2])
3. Now apply a convolution with a circular disc, \(B = D \ast B\), where D is the disc kernel. disc = [cv.getStructuringElement](../../d4/d86/group__imgproc__filter.html#gac342a1bb6eabf6f55c803b09268e36dc "../../d4/d86/group__imgproc__filter.html#gac342a1bb6eabf6f55c803b09268e36dc")(cv.MORPH\_ELLIPSE,(5,5))[cv.filter2D](../../d4/d86/group__imgproc__filter.html#ga27c049795ce870216ddfb366086b5a04 "../../d4/d86/group__imgproc__filter.html#ga27c049795ce870216ddfb366086b5a04")(B,-1,disc,B)B = np.uint8(B)[cv.normalize](../../d2/de8/group__core__array.html#ga7bcf47a1df78cf575162e0aed44960cb "../../d2/de8/group__core__array.html#ga7bcf47a1df78cf575162e0aed44960cb")(B,B,0,255,cv.NORM\_MINMAX)
4. Now the location of maximum intensity gives us the location of object. If we are expecting a region in the image, thresholding for a suitable value gives a nice result. ret,thresh = [cv.threshold](../../d7/d1b/group__imgproc__misc.html#gae8a4a146d1ca78c626a53577199e9c57 "../../d7/d1b/group__imgproc__misc.html#gae8a4a146d1ca78c626a53577199e9c57")(B,50,255,0) That's it !!

## Backprojection in OpenCV

OpenCV provides an inbuilt function **[cv.calcBackProject()](../../d6/dc7/group__imgproc__hist.html#ga3a0af640716b456c3d14af8aee12e3ca "Calculates the back projection of a histogram. ")**. Its parameters are almost same as the **[cv.calcHist()](../../d6/dc7/group__imgproc__hist.html#ga4b2b5fd75503ff9e6844cc4dcdaed35d "Calculates a histogram of a set of arrays. ")** function. One of its parameter is histogram which is histogram of the object and we have to find it. Also, the object histogram should be normalized before passing on to the backproject function. It returns the probability image. Then we convolve the image with a disc kernel and apply threshold. Below is my code and output : 

import numpy as npimport cv2 as cvroi = [cv.imread](../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56 "../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56")('rose\_red.png')assert roi is not None, "file could not be read, check with os.path.exists()"hsv = [cv.cvtColor](../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab "../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab")(roi,cv.COLOR\_BGR2HSV)target = [cv.imread](../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56 "../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56")('rose.png')assert target is not None, "file could not be read, check with os.path.exists()"hsvt = [cv.cvtColor](../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab "../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab")(target,cv.COLOR\_BGR2HSV)# calculating object histogramroihist = [cv.calcHist](../../d6/dc7/group__imgproc__hist.html#ga6ca1876785483836f72a77ced8ea759a "../../d6/dc7/group__imgproc__hist.html#ga6ca1876785483836f72a77ced8ea759a")([hsv],[0, 1], None, [180, 256], [0, 180, 0, 256] )# normalize histogram and apply backprojection[cv.normalize](../../d2/de8/group__core__array.html#ga7bcf47a1df78cf575162e0aed44960cb "../../d2/de8/group__core__array.html#ga7bcf47a1df78cf575162e0aed44960cb")(roihist,roihist,0,255,cv.NORM\_MINMAX)dst = [cv.calcBackProject](../../d6/dc7/group__imgproc__hist.html#gab644bc90e7475cc047aa1b25dbcbd8df "../../d6/dc7/group__imgproc__hist.html#gab644bc90e7475cc047aa1b25dbcbd8df")([hsvt],[0,1],roihist,[0,180,0,256],1)# Now convolute with circular discdisc = [cv.getStructuringElement](../../d4/d86/group__imgproc__filter.html#gac342a1bb6eabf6f55c803b09268e36dc "../../d4/d86/group__imgproc__filter.html#gac342a1bb6eabf6f55c803b09268e36dc")(cv.MORPH\_ELLIPSE,(5,5))[cv.filter2D](../../d4/d86/group__imgproc__filter.html#ga27c049795ce870216ddfb366086b5a04 "../../d4/d86/group__imgproc__filter.html#ga27c049795ce870216ddfb366086b5a04")(dst,-1,disc,dst)# threshold and binary ANDret,thresh = [cv.threshold](../../d7/d1b/group__imgproc__misc.html#gae8a4a146d1ca78c626a53577199e9c57 "../../d7/d1b/group__imgproc__misc.html#gae8a4a146d1ca78c626a53577199e9c57")(dst,50,255,0)thresh = [cv.merge](../../d2/de8/group__core__array.html#ga61f2f2bde4a0a0154b2333ea504fab1d "../../d2/de8/group__core__array.html#ga61f2f2bde4a0a0154b2333ea504fab1d")((thresh,thresh,thresh))res = [cv.bitwise\_and](../../d2/de8/group__core__array.html#ga60b4d04b251ba5eb1392c34425497e14 "../../d2/de8/group__core__array.html#ga60b4d04b251ba5eb1392c34425497e14")(target,thresh)res = np.vstack((target,thresh,res))[cv.imwrite](../../d4/da8/group__imgcodecs.html#gabbc7ef1aa2edfaa87772f1202d67e0ce "../../d4/da8/group__imgcodecs.html#gabbc7ef1aa2edfaa87772f1202d67e0ce")('res.jpg',res) Below is one example I worked with. I used the region inside blue rectangle as sample object and I wanted to extract the full ground.

![backproject_opencv.jpg](../../backproject_opencv.jpg)

image
## Additional Resources

1. "Indexing via color histograms", Swain, Michael J. , Third international conference on computer vision,1990.

## Exercises

---

Generated on Wed Oct 4 2023 23:37:11 for OpenCV by  [![doxygen](../../doxygen.png)](http://www.doxygen.org/index.html "http://www.doxygen.org/index.html") 1.8.13

//<![CDATA[
addTutorialsButtons();
//]]>

