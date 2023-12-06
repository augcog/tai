

OpenCV: Image Processing in OpenCV

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

Image Processing in OpenCV  

* [Changing Colorspaces](../../df/d9d/tutorial_py_colorspaces.html "../../df/d9d/tutorial_py_colorspaces.html")

Learn to change images between different color spaces. Plus learn to track a colored object in a video.
* [Geometric Transformations of Images](../../da/d6e/tutorial_py_geometric_transformations.html "../../da/d6e/tutorial_py_geometric_transformations.html")

Learn to apply different geometric transformations to images like rotation, translation etc.
* [Image Thresholding](../../d7/d4d/tutorial_py_thresholding.html "../../d7/d4d/tutorial_py_thresholding.html")

Learn to convert images to binary images using global thresholding, Adaptive thresholding, Otsu's binarization etc
* [Smoothing Images](../../d4/d13/tutorial_py_filtering.html "../../d4/d13/tutorial_py_filtering.html")

Learn to blur the images, filter the images with custom kernels etc.
* [Morphological Transformations](../../d9/d61/tutorial_py_morphological_ops.html "../../d9/d61/tutorial_py_morphological_ops.html")

Learn about morphological transformations like Erosion, Dilation, Opening, Closing etc
* [Image Gradients](../../d5/d0f/tutorial_py_gradients.html "../../d5/d0f/tutorial_py_gradients.html")

Learn to find image gradients, edges etc.
* [Canny Edge Detection](../../da/d22/tutorial_py_canny.html "../../da/d22/tutorial_py_canny.html")

Learn to find edges with Canny Edge Detection
* [Image Pyramids](../../dc/dff/tutorial_py_pyramids.html "../../dc/dff/tutorial_py_pyramids.html")

Learn about image pyramids and how to use them for image blending
* [Contours in OpenCV](../../d3/d05/tutorial_py_table_of_contents_contours.html "../../d3/d05/tutorial_py_table_of_contents_contours.html")

All about Contours in OpenCV
* [Histograms in OpenCV](../../de/db2/tutorial_py_table_of_contents_histograms.html "../../de/db2/tutorial_py_table_of_contents_histograms.html")

All about histograms in OpenCV
* [Image Transforms in OpenCV](../../dd/dc4/tutorial_py_table_of_contents_transforms.html "../../dd/dc4/tutorial_py_table_of_contents_transforms.html")

Meet different Image Transforms in OpenCV like Fourier Transform, Cosine Transform etc.
* [Template Matching](../../d4/dc6/tutorial_py_template_matching.html "../../d4/dc6/tutorial_py_template_matching.html")

Learn to search for an object in an image using Template Matching
* [Hough Line Transform](../../d6/d10/tutorial_py_houghlines.html "../../d6/d10/tutorial_py_houghlines.html")

Learn to detect lines in an image
* [Hough Circle Transform](../../da/d53/tutorial_py_houghcircles.html "../../da/d53/tutorial_py_houghcircles.html")

Learn to detect circles in an image
* [Image Segmentation with Watershed Algorithm](../../d3/db4/tutorial_py_watershed.html "../../d3/db4/tutorial_py_watershed.html")

Learn to segment images with watershed segmentation
* [Interactive Foreground Extraction using GrabCut Algorithm](../../d8/d83/tutorial_py_grabcut.html "../../d8/d83/tutorial_py_grabcut.html")

Learn to extract foreground with GrabCut algorithm

---

Generated on Wed Oct 4 2023 23:37:11 for OpenCV by  [![doxygen](../../doxygen.png)](http://www.doxygen.org/index.html "http://www.doxygen.org/index.html") 1.8.13

//<![CDATA[
addTutorialsButtons();
//]]>

