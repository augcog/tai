

OpenCV: Histograms - 2: Histogram Equalization

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

Histograms - 2: Histogram Equalization  

## Goal

In this section,

* We will learn the concepts of histogram equalization and use it to improve the contrast of our images.

## Theory

Consider an image whose pixel values are confined to some specific range of values only. For eg, brighter image will have all pixels confined to high values. But a good image will have pixels from all regions of the image. So you need to stretch this histogram to either ends (as given in below image, from wikipedia) and that is what Histogram Equalization does (in simple words). This normally improves the contrast of the image.

![histogram_equalization.png](../../histogram_equalization.png)

image
 I would recommend you to read the wikipedia page on [Histogram Equalization](https://en.wikipedia.org/wiki/Histogram_equalization "https://en.wikipedia.org/wiki/Histogram_equalization") for more details about it. It has a very good explanation with worked out examples, so that you would understand almost everything after reading that. Instead, here we will see its Numpy implementation. After that, we will see OpenCV function. 

import numpy as npimport cv2 as cvfrom matplotlib import pyplot as pltimg = [cv.imread](../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56 "../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56")('wiki.jpg', cv.IMREAD\_GRAYSCALE)assert img is not None, "file could not be read, check with os.path.exists()"hist,bins = np.histogram(img.flatten(),256,[0,256])cdf = hist.cumsum()cdf\_normalized = cdf \* float(hist.max()) / cdf.max()plt.plot(cdf\_normalized, color = 'b')plt.hist(img.flatten(),256,[0,256], color = 'r')plt.xlim([0,256])plt.legend(('cdf','histogram'), loc = 'upper left')plt.show() 
![histeq_numpy1.jpg](../../histeq_numpy1.jpg)

image
 You can see histogram lies in brighter region. We need the full spectrum. For that, we need a transformation function which maps the input pixels in brighter region to output pixels in full region. That is what histogram equalization does.

Now we find the minimum histogram value (excluding 0) and apply the histogram equalization equation as given in wiki page. But I have used here, the masked array concept array from Numpy. For masked array, all operations are performed on non-masked elements. You can read more about it from Numpy docs on masked arrays. 

cdf\_m = np.ma.masked\_equal(cdf,0)cdf\_m = (cdf\_m - cdf\_m.min())\*255/(cdf\_m.max()-cdf\_m.min())cdf = np.ma.filled(cdf\_m,0).astype('uint8') Now we have the look-up table that gives us the information on what is the output pixel value for every input pixel value. So we just apply the transform. 

img2 = cdf[img] Now we calculate its histogram and cdf as before ( you do it) and result looks like below :

![histeq_numpy2.jpg](../../histeq_numpy2.jpg)

image
 Another important feature is that, even if the image was a darker image (instead of a brighter one we used), after equalization we will get almost the same image as we got. As a result, this is used as a "reference tool" to make all images with same lighting conditions. This is useful in many cases. For example, in face recognition, before training the face data, the images of faces are histogram equalized to make them all with same lighting conditions.

## Histograms Equalization in OpenCV

OpenCV has a function to do this, **[cv.equalizeHist()](../../d6/dc7/group__imgproc__hist.html#ga7e54091f0c937d49bf84152a16f76d6e "Equalizes the histogram of a grayscale image. ")**. Its input is just grayscale image and output is our histogram equalized image.

Below is a simple code snippet showing its usage for same image we used : 

img = [cv.imread](../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56 "../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56")('wiki.jpg', cv.IMREAD\_GRAYSCALE)assert img is not None, "file could not be read, check with os.path.exists()"equ = [cv.equalizeHist](../../d6/dc7/group__imgproc__hist.html#ga7e54091f0c937d49bf84152a16f76d6e "../../d6/dc7/group__imgproc__hist.html#ga7e54091f0c937d49bf84152a16f76d6e")(img)res = np.hstack((img,equ)) #stacking images side-by-side[cv.imwrite](../../d4/da8/group__imgcodecs.html#gabbc7ef1aa2edfaa87772f1202d67e0ce "../../d4/da8/group__imgcodecs.html#gabbc7ef1aa2edfaa87772f1202d67e0ce")('res.png',res) 
![equalization_opencv.jpg](../../equalization_opencv.jpg)

image
 So now you can take different images with different light conditions, equalize it and check the results.

Histogram equalization is good when histogram of the image is confined to a particular region. It won't work good in places where there is large intensity variations where histogram covers a large region, ie both bright and dark pixels are present. Please check the SOF links in Additional Resources.

## CLAHE (Contrast Limited Adaptive Histogram Equalization)

The first histogram equalization we just saw, considers the global contrast of the image. In many cases, it is not a good idea. For example, below image shows an input image and its result after global histogram equalization.

![clahe_1.jpg](../../clahe_1.jpg)

image
 It is true that the background contrast has improved after histogram equalization. But compare the face of statue in both images. We lost most of the information there due to over-brightness. It is because its histogram is not confined to a particular region as we saw in previous cases (Try to plot histogram of input image, you will get more intuition).

So to solve this problem, **adaptive histogram equalization** is used. In this, image is divided into small blocks called "tiles" (tileSize is 8x8 by default in OpenCV). Then each of these blocks are histogram equalized as usual. So in a small area, histogram would confine to a small region (unless there is noise). If noise is there, it will be amplified. To avoid this, **contrast limiting** is applied. If any histogram bin is above the specified contrast limit (by default 40 in OpenCV), those pixels are clipped and distributed uniformly to other bins before applying histogram equalization. After equalization, to remove artifacts in tile borders, bilinear interpolation is applied.

Below code snippet shows how to apply CLAHE in OpenCV: 

import numpy as npimport cv2 as cvimg = [cv.imread](../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56 "../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56")('tsukuba\_l.png', cv.IMREAD\_GRAYSCALE)assert img is not None, "file could not be read, check with os.path.exists()"# create a CLAHE object (Arguments are optional).clahe = [cv.createCLAHE](../../d6/dc7/group__imgproc__hist.html#gad689d2607b7b3889453804f414ab1018 "../../d6/dc7/group__imgproc__hist.html#gad689d2607b7b3889453804f414ab1018")(clipLimit=2.0, tileGridSize=(8,8))cl1 = clahe.apply(img)[cv.imwrite](../../d4/da8/group__imgcodecs.html#gabbc7ef1aa2edfaa87772f1202d67e0ce "../../d4/da8/group__imgcodecs.html#gabbc7ef1aa2edfaa87772f1202d67e0ce")('clahe\_2.jpg',cl1) See the result below and compare it with results above, especially the statue region:

![clahe_2.jpg](../../clahe_2.jpg)

image
## Additional Resources

1. Wikipedia page on [Histogram Equalization](https://en.wikipedia.org/wiki/Histogram_equalization "https://en.wikipedia.org/wiki/Histogram_equalization")
2. [Masked Arrays in Numpy](http://docs.scipy.org/doc/numpy/reference/maskedarray.html "http://docs.scipy.org/doc/numpy/reference/maskedarray.html")

Also check these SOF questions regarding contrast adjustment:

1. [How can I adjust contrast in OpenCV in C?](http://stackoverflow.com/questions/10549245/how-can-i-adjust-contrast-in-opencv-in-c "http://stackoverflow.com/questions/10549245/how-can-i-adjust-contrast-in-opencv-in-c")
2. [How do I equalize contrast & brightness of images using opencv?](http://stackoverflow.com/questions/10561222/how-do-i-equalize-contrast-brightness-of-images-using-opencv "http://stackoverflow.com/questions/10561222/how-do-i-equalize-contrast-brightness-of-images-using-opencv")

## Exercises

---

Generated on Wed Oct 4 2023 23:37:11 for OpenCV by  [![doxygen](../../doxygen.png)](http://www.doxygen.org/index.html "http://www.doxygen.org/index.html") 1.8.13

//<![CDATA[
addTutorialsButtons();
//]]>

