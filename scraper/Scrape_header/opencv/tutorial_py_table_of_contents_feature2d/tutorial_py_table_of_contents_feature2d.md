

OpenCV: Feature Detection and Description

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

Feature Detection and Description  

* [Understanding Features](../../df/d54/tutorial_py_features_meaning.html "../../df/d54/tutorial_py_features_meaning.html")

What are the main features in an image? How can finding those features be useful to us?
* [Harris Corner Detection](../../dc/d0d/tutorial_py_features_harris.html "../../dc/d0d/tutorial_py_features_harris.html")

Okay, Corners are good features? But how do we find them?
* [Shi-Tomasi Corner Detector & Good Features to Track](../../d4/d8c/tutorial_py_shi_tomasi.html "../../d4/d8c/tutorial_py_shi_tomasi.html")

We will look into Shi-Tomasi corner detection
* [Introduction to SIFT (Scale-Invariant Feature Transform)](../../da/df5/tutorial_py_sift_intro.html "../../da/df5/tutorial_py_sift_intro.html")

Harris corner detector is not good enough when scale of image changes. Lowe developed a breakthrough method to find scale-invariant features and it is called SIFT
* [Introduction to SURF (Speeded-Up Robust Features)](../../df/dd2/tutorial_py_surf_intro.html "../../df/dd2/tutorial_py_surf_intro.html")

SIFT is really good, but not fast enough, so people came up with a speeded-up version called SURF.
* [FAST Algorithm for Corner Detection](../../df/d0c/tutorial_py_fast.html "../../df/d0c/tutorial_py_fast.html")

All the above feature detection methods are good in some way. But they are not fast enough to work in real-time applications like SLAM. There comes the FAST algorithm, which is really "FAST".
* [BRIEF (Binary Robust Independent Elementary Features)](../../dc/d7d/tutorial_py_brief.html "../../dc/d7d/tutorial_py_brief.html")

SIFT uses a feature descriptor with 128 floating point numbers. Consider thousands of such features. It takes lots of memory and more time for matching. We can compress it to make it faster. But still we have to calculate it first. There comes BRIEF which gives the shortcut to find binary descriptors with less memory, faster matching, still higher recognition rate.
* [ORB (Oriented FAST and Rotated BRIEF)](../../d1/d89/tutorial_py_orb.html "../../d1/d89/tutorial_py_orb.html")

SIFT and SURF are good in what they do, but what if you have to pay a few dollars every year to use them in your applications? Yeah, they are patented!!! To solve that problem, OpenCV devs came up with a new "FREE" alternative to SIFT & SURF, and that is ORB.
* [Feature Matching](../../dc/dc3/tutorial_py_matcher.html "../../dc/dc3/tutorial_py_matcher.html")

We know a great deal about feature detectors and descriptors. It is time to learn how to match different descriptors. OpenCV provides two techniques, Brute-Force matcher and FLANN based matcher.
* [Feature Matching + Homography to find Objects](../../d1/de0/tutorial_py_feature_homography.html "../../d1/de0/tutorial_py_feature_homography.html")

Now we know about feature matching. Let's mix it up with calib3d module to find objects in a complex image.

---

Generated on Wed Oct 4 2023 23:37:11 for OpenCV by  [![doxygen](../../doxygen.png)](http://www.doxygen.org/index.html "http://www.doxygen.org/index.html") 1.8.13

//<![CDATA[
addTutorialsButtons();
//]]>

