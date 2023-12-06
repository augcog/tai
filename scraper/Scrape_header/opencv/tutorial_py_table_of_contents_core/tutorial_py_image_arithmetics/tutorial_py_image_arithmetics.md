

OpenCV: Arithmetic Operations on Images

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
* [Core Operations](../../d7/d16/tutorial_py_table_of_contents_core.html "../../d7/d16/tutorial_py_table_of_contents_core.html")

Arithmetic Operations on Images  

## Goal

* Learn several arithmetic operations on images, like addition, subtraction, bitwise operations, and etc.
* Learn these functions: **[cv.add()](../../d2/de8/group__core__array.html#ga10ac1bfb180e2cfda1701d06c24fdbd6 "Calculates the per-element sum of two arrays or an array and a scalar. ")**, **[cv.addWeighted()](../../d2/de8/group__core__array.html#gafafb2513349db3bcff51f54ee5592a19 "Calculates the weighted sum of two arrays. ")**, etc.

## Image Addition

You can add two images with the OpenCV function, [cv.add()](../../d2/de8/group__core__array.html#ga10ac1bfb180e2cfda1701d06c24fdbd6 "Calculates the per-element sum of two arrays or an array and a scalar. "), or simply by the numpy operation res = img1 + img2. Both images should be of same depth and type, or the second image can just be a scalar value.

NoteThere is a difference between OpenCV addition and Numpy addition. OpenCV addition is a saturated operation while Numpy addition is a modulo operation.
For example, consider the below sample: 

>>> x = np.uint8([250])>>> y = np.uint8([10])>>> [print](../../df/d57/namespacecv_1_1dnn.html#a43417dcaeb3c1e2a09b9d948e234c366 "../../df/d57/namespacecv_1_1dnn.html#a43417dcaeb3c1e2a09b9d948e234c366")( [cv.add](../../d2/de8/group__core__array.html#ga10ac1bfb180e2cfda1701d06c24fdbd6 "../../d2/de8/group__core__array.html#ga10ac1bfb180e2cfda1701d06c24fdbd6")(x,y) ) # 250+10 = 260 => 255[[255]]>>> [print](../../df/d57/namespacecv_1_1dnn.html#a43417dcaeb3c1e2a09b9d948e234c366 "../../df/d57/namespacecv_1_1dnn.html#a43417dcaeb3c1e2a09b9d948e234c366")( x+y ) # 250+10 = 260 % 256 = 4[4] This will be more visible when you add two images. Stick with OpenCV functions, because they will provide a better result.

## Image Blending

This is also image addition, but different weights are given to images in order to give a feeling of blending or transparency. Images are added as per the equation below:

\[g(x) = (1 - \alpha)f\_{0}(x) + \alpha f\_{1}(x)\]

By varying \(\alpha\) from \(0 \rightarrow 1\), you can perform a cool transition between one image to another.

Here I took two images to blend together. The first image is given a weight of 0.7 and the second image is given 0.3. [cv.addWeighted()](../../d2/de8/group__core__array.html#gafafb2513349db3bcff51f54ee5592a19 "Calculates the weighted sum of two arrays. ") applies the following equation to the image:

\[dst = \alpha \cdot img1 + \beta \cdot img2 + \gamma\]

Here \(\gamma\) is taken as zero. 

img1 = [cv.imread](../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56 "../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56")('ml.png')img2 = [cv.imread](../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56 "../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56")('opencv-logo.png')assert img1 is not None, "file could not be read, check with os.path.exists()"assert img2 is not None, "file could not be read, check with os.path.exists()"dst = [cv.addWeighted](../../d2/de8/group__core__array.html#gafafb2513349db3bcff51f54ee5592a19 "../../d2/de8/group__core__array.html#gafafb2513349db3bcff51f54ee5592a19")(img1,0.7,img2,0.3,0)[cv.imshow](../../df/d24/group__highgui__opengl.html#gaae7e90aa3415c68dba22a5ff2cefc25d "../../df/d24/group__highgui__opengl.html#gaae7e90aa3415c68dba22a5ff2cefc25d")('dst',dst)[cv.waitKey](../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7 "../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7")(0)[cv.destroyAllWindows](../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481 "../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481")() Check the result below:

![blending.jpg](../../blending.jpg)

image
## Bitwise Operations

This includes the bitwise AND, OR, NOT, and XOR operations. They will be highly useful while extracting any part of the image (as we will see in coming chapters), defining and working with non-rectangular ROI's, and etc. Below we will see an example of how to change a particular region of an image.

I want to put the OpenCV logo above an image. If I add two images, it will change the color. If I blend them, I get a transparent effect. But I want it to be opaque. If it was a rectangular region, I could use ROI as we did in the last chapter. But the OpenCV logo is a not a rectangular shape. So you can do it with bitwise operations as shown below: 

# Load two imagesimg1 = [cv.imread](../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56 "../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56")('messi5.jpg')img2 = [cv.imread](../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56 "../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56")('opencv-logo-white.png')assert img1 is not None, "file could not be read, check with os.path.exists()"assert img2 is not None, "file could not be read, check with os.path.exists()"# I want to put logo on top-left corner, So I create a ROIrows,cols,channels = img2.shaperoi = img1[0:rows, 0:cols]# Now create a mask of logo and create its inverse mask alsoimg2gray = [cv.cvtColor](../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab "../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab")(img2,cv.COLOR\_BGR2GRAY)ret, mask = [cv.threshold](../../d7/d1b/group__imgproc__misc.html#gae8a4a146d1ca78c626a53577199e9c57 "../../d7/d1b/group__imgproc__misc.html#gae8a4a146d1ca78c626a53577199e9c57")(img2gray, 10, 255, cv.THRESH\_BINARY)mask\_inv = [cv.bitwise\_not](../../d2/de8/group__core__array.html#ga0002cf8b418479f4cb49a75442baee2f "../../d2/de8/group__core__array.html#ga0002cf8b418479f4cb49a75442baee2f")(mask)# Now black-out the area of logo in ROIimg1\_bg = [cv.bitwise\_and](../../d2/de8/group__core__array.html#ga60b4d04b251ba5eb1392c34425497e14 "../../d2/de8/group__core__array.html#ga60b4d04b251ba5eb1392c34425497e14")(roi,roi,mask = mask\_inv)# Take only region of logo from logo image.img2\_fg = [cv.bitwise\_and](../../d2/de8/group__core__array.html#ga60b4d04b251ba5eb1392c34425497e14 "../../d2/de8/group__core__array.html#ga60b4d04b251ba5eb1392c34425497e14")(img2,img2,mask = mask)# Put logo in ROI and modify the main imagedst = [cv.add](../../d2/de8/group__core__array.html#ga10ac1bfb180e2cfda1701d06c24fdbd6 "../../d2/de8/group__core__array.html#ga10ac1bfb180e2cfda1701d06c24fdbd6")(img1\_bg,img2\_fg)img1[0:rows, 0:cols ] = dst[cv.imshow](../../df/d24/group__highgui__opengl.html#gaae7e90aa3415c68dba22a5ff2cefc25d "../../df/d24/group__highgui__opengl.html#gaae7e90aa3415c68dba22a5ff2cefc25d")('res',img1)[cv.waitKey](../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7 "../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7")(0)[cv.destroyAllWindows](../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481 "../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481")() See the result below. Left image shows the mask we created. Right image shows the final result. For more understanding, display all the intermediate images in the above code, especially img1\_bg and img2\_fg.

![overlay.jpg](../../overlay.jpg)

image
## Additional Resources

## Exercises

1. Create a slide show of images in a folder with smooth transition between images using [cv.addWeighted](../../d2/de8/group__core__array.html#gafafb2513349db3bcff51f54ee5592a19 "Calculates the weighted sum of two arrays. ") function

---

Generated on Wed Oct 4 2023 23:37:11 for OpenCV by  [![doxygen](../../doxygen.png)](http://www.doxygen.org/index.html "http://www.doxygen.org/index.html") 1.8.13

//<![CDATA[
addTutorialsButtons();
//]]>

