

OpenCV: OpenCV-Python Tutorials

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

OpenCV-Python Tutorials  

* [Introduction to OpenCV](../../da/df6/tutorial_py_table_of_contents_setup.html "../../da/df6/tutorial_py_table_of_contents_setup.html")

Learn how to setup OpenCV-Python on your computer!
* [Gui Features in OpenCV](../../dc/d4d/tutorial_py_table_of_contents_gui.html "../../dc/d4d/tutorial_py_table_of_contents_gui.html")

Here you will learn how to display and save images and videos, control mouse events and create trackbar.
* [Core Operations](../../d7/d16/tutorial_py_table_of_contents_core.html "../../d7/d16/tutorial_py_table_of_contents_core.html")

In this section you will learn basic operations on image like pixel editing, geometric transformations, code optimization, some mathematical tools etc.
* [Image Processing in OpenCV](../../d2/d96/tutorial_py_table_of_contents_imgproc.html "../../d2/d96/tutorial_py_table_of_contents_imgproc.html")

In this section you will learn different image processing functions inside OpenCV.
* [Feature Detection and Description](../../db/d27/tutorial_py_table_of_contents_feature2d.html "../../db/d27/tutorial_py_table_of_contents_feature2d.html")

In this section you will learn about feature detectors and descriptors
* [Video analysis (video module)](../../da/dd0/tutorial_table_of_content_video.html "../../da/dd0/tutorial_table_of_content_video.html")

In this section you will learn different techniques to work with videos like object tracking etc.
* [Camera Calibration and 3D Reconstruction](../../d9/db7/tutorial_py_table_of_contents_calib3d.html "../../d9/db7/tutorial_py_table_of_contents_calib3d.html")

In this section we will learn about camera calibration, stereo imaging etc.
* [Machine Learning](../../d6/de2/tutorial_py_table_of_contents_ml.html "../../d6/de2/tutorial_py_table_of_contents_ml.html")

In this section you will learn different image processing functions inside OpenCV.
* [Computational Photography](../../d0/d07/tutorial_py_table_of_contents_photo.html "../../d0/d07/tutorial_py_table_of_contents_photo.html")

In this section you will learn different computational photography techniques like image denoising etc.
* [Object Detection (objdetect module)](../../d2/d64/tutorial_table_of_content_objdetect.html "../../d2/d64/tutorial_table_of_content_objdetect.html")

In this section you will learn object detection techniques like face detection etc.
* [OpenCV-Python Bindings](../../df/da2/tutorial_py_table_of_contents_bindings.html "../../df/da2/tutorial_py_table_of_contents_bindings.html")

In this section, we will see how OpenCV-Python bindings are generated

---

Generated on Wed Oct 4 2023 23:37:12 for OpenCV by  [![doxygen](../../doxygen.png)](http://www.doxygen.org/index.html "http://www.doxygen.org/index.html") 1.8.13

//<![CDATA[
addTutorialsButtons();
//]]>

