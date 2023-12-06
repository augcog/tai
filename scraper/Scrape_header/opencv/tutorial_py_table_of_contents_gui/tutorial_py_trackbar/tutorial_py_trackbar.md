

OpenCV: Trackbar as the Color Palette

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
* [Gui Features in OpenCV](../../dc/d4d/tutorial_py_table_of_contents_gui.html "../../dc/d4d/tutorial_py_table_of_contents_gui.html")

Trackbar as the Color Palette  

## Goal

* Learn to bind trackbar to OpenCV windows
* You will learn these functions : **[cv.getTrackbarPos()](../../d7/dfc/group__highgui.html#ga122632e9e91b9ec06943472c55d9cda8 "Returns the trackbar position. ")**, **[cv.createTrackbar()](../../d7/dfc/group__highgui.html#gaf78d2155d30b728fc413803745b67a9b "Creates a trackbar and attaches it to the specified window. ")** etc.

## Code Demo

Here we will create a simple application which shows the color you specify. You have a window which shows the color and three trackbars to specify each of B,G,R colors. You slide the trackbar and correspondingly window color changes. By default, initial color will be set to Black.

For [cv.createTrackbar()](../../d7/dfc/group__highgui.html#gaf78d2155d30b728fc413803745b67a9b "Creates a trackbar and attaches it to the specified window. ") function, first argument is the trackbar name, second one is the window name to which it is attached, third argument is the default value, fourth one is the maximum value and fifth one is the callback function which is executed every time trackbar value changes. The callback function always has a default argument which is the trackbar position. In our case, function does nothing, so we simply pass.

Another important application of trackbar is to use it as a button or switch. OpenCV, by default, doesn't have button functionality. So you can use trackbar to get such functionality. In our application, we have created one switch in which application works only if switch is ON, otherwise screen is always black. 

import numpy as npimport cv2 as cvdef nothing(x): pass# Create a black image, a windowimg = np.zeros((300,512,3), np.uint8)[cv.namedWindow](../../d7/dfc/group__highgui.html#ga5afdf8410934fd099df85c75b2e0888b "../../d7/dfc/group__highgui.html#ga5afdf8410934fd099df85c75b2e0888b")('image')# create trackbars for color change[cv.createTrackbar](../../d7/dfc/group__highgui.html#gaf78d2155d30b728fc413803745b67a9b "../../d7/dfc/group__highgui.html#gaf78d2155d30b728fc413803745b67a9b")('R','image',0,255,nothing)[cv.createTrackbar](../../d7/dfc/group__highgui.html#gaf78d2155d30b728fc413803745b67a9b "../../d7/dfc/group__highgui.html#gaf78d2155d30b728fc413803745b67a9b")('G','image',0,255,nothing)[cv.createTrackbar](../../d7/dfc/group__highgui.html#gaf78d2155d30b728fc413803745b67a9b "../../d7/dfc/group__highgui.html#gaf78d2155d30b728fc413803745b67a9b")('B','image',0,255,nothing)# create switch for ON/OFF functionalityswitch = '0 : OFF \n1 : ON'[cv.createTrackbar](../../d7/dfc/group__highgui.html#gaf78d2155d30b728fc413803745b67a9b "../../d7/dfc/group__highgui.html#gaf78d2155d30b728fc413803745b67a9b")(switch, 'image',0,1,nothing)while(1): [cv.imshow](../../df/d24/group__highgui__opengl.html#gaae7e90aa3415c68dba22a5ff2cefc25d "../../df/d24/group__highgui__opengl.html#gaae7e90aa3415c68dba22a5ff2cefc25d")('image',img) k = [cv.waitKey](../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7 "../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7")(1) & 0xFF if k == 27: break # get current positions of four trackbars r = [cv.getTrackbarPos](../../d7/dfc/group__highgui.html#ga122632e9e91b9ec06943472c55d9cda8 "../../d7/dfc/group__highgui.html#ga122632e9e91b9ec06943472c55d9cda8")('R','image') g = [cv.getTrackbarPos](../../d7/dfc/group__highgui.html#ga122632e9e91b9ec06943472c55d9cda8 "../../d7/dfc/group__highgui.html#ga122632e9e91b9ec06943472c55d9cda8")('G','image') b = [cv.getTrackbarPos](../../d7/dfc/group__highgui.html#ga122632e9e91b9ec06943472c55d9cda8 "../../d7/dfc/group__highgui.html#ga122632e9e91b9ec06943472c55d9cda8")('B','image') s = [cv.getTrackbarPos](../../d7/dfc/group__highgui.html#ga122632e9e91b9ec06943472c55d9cda8 "../../d7/dfc/group__highgui.html#ga122632e9e91b9ec06943472c55d9cda8")(switch,'image') if s == 0: img[:] = 0 else: img[:] = [b,g,r][cv.destroyAllWindows](../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481 "../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481")() The screenshot of the application looks like below :

![trackbar_screenshot.jpg](../../trackbar_screenshot.jpg)

image
## Exercises

1. Create a Paint application with adjustable colors and brush radius using trackbars. For drawing, refer previous tutorial on mouse handling.

---

Generated on Wed Oct 4 2023 23:37:11 for OpenCV by  [![doxygen](../../doxygen.png)](http://www.doxygen.org/index.html "http://www.doxygen.org/index.html") 1.8.13

//<![CDATA[
addTutorialsButtons();
//]]>

