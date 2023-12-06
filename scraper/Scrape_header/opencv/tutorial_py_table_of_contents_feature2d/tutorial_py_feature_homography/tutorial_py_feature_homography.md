

OpenCV: Feature Matching + Homography to find Objects

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

Feature Matching + Homography to find Objects  

## Goal

In this chapter,

* We will mix up the feature matching and findHomography from calib3d module to find known objects in a complex image.

## Basics

So what we did in last session? We used a queryImage, found some feature points in it, we took another trainImage, found the features in that image too and we found the best matches among them. In short, we found locations of some parts of an object in another cluttered image. This information is sufficient to find the object exactly on the trainImage.

For that, we can use a function from calib3d module, ie **[cv.findHomography()](../../d9/d0c/group__calib3d.html#ga4abc2ece9fab9398f2e560d53c8c9780 "Finds a perspective transformation between two planes. ")**. If we pass the set of points from both the images, it will find the perspective transformation of that object. Then we can use **[cv.perspectiveTransform()](../../d2/de8/group__core__array.html#gad327659ac03e5fd6894b90025e6900a7 "Performs the perspective matrix transformation of vectors. ")** to find the object. It needs at least four correct points to find the transformation.

We have seen that there can be some possible errors while matching which may affect the result. To solve this problem, algorithm uses RANSAC or LEAST\_MEDIAN (which can be decided by the flags). So good matches which provide correct estimation are called inliers and remaining are called outliers. **[cv.findHomography()](../../d9/d0c/group__calib3d.html#ga4abc2ece9fab9398f2e560d53c8c9780 "Finds a perspective transformation between two planes. ")** returns a mask which specifies the inlier and outlier points.

So let's do it !!!

## Code

First, as usual, let's find SIFT features in images and apply the ratio test to find the best matches. 

import numpy as npimport cv2 as cvfrom matplotlib import pyplot as pltMIN\_MATCH\_COUNT = 10img1 = [cv.imread](../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56 "../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56")('box.png', cv.IMREAD\_GRAYSCALE) # queryImageimg2 = [cv.imread](../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56 "../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56")('box\_in\_scene.png', cv.IMREAD\_GRAYSCALE) # trainImage# Initiate SIFT detectorsift = cv.SIFT\_create()# find the keypoints and descriptors with SIFTkp1, des1 = sift.detectAndCompute(img1,None)kp2, des2 = sift.detectAndCompute(img2,None)FLANN\_INDEX\_KDTREE = 1index\_params = dict(algorithm = FLANN\_INDEX\_KDTREE, trees = 5)search\_params = dict(checks = 50)flann = [cv.FlannBasedMatcher](../../dc/de2/classcv_1_1FlannBasedMatcher.html "../../dc/de2/classcv_1_1FlannBasedMatcher.html")(index\_params, search\_params)matches = flann.knnMatch(des1,des2,k=2)# store all the good matches as per Lowe's ratio test.good = []for m,n in matches: if m.distance < 0.7\*n.distance: good.append(m) Now we set a condition that at least 10 matches (defined by MIN\_MATCH\_COUNT) are to be there to find the object. Otherwise simply show a message saying not enough matches are present.

If enough matches are found, we extract the locations of matched keypoints in both the images. They are passed to find the perspective transformation. Once we get this 3x3 transformation matrix, we use it to transform the corners of queryImage to corresponding points in trainImage. Then we draw it. 

if len(good)>MIN\_MATCH\_COUNT: src\_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2) dst\_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2) M, mask = [cv.findHomography](../../d9/d0c/group__calib3d.html#ga4b3841447530523e5272ec05c5d1e411 "../../d9/d0c/group__calib3d.html#ga4b3841447530523e5272ec05c5d1e411")(src\_pts, dst\_pts, cv.RANSAC,5.0) matchesMask = mask.ravel().tolist() h,w = img1.shape pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2) dst = [cv.perspectiveTransform](../../d2/de8/group__core__array.html#gad327659ac03e5fd6894b90025e6900a7 "../../d2/de8/group__core__array.html#gad327659ac03e5fd6894b90025e6900a7")(pts,M) img2 = [cv.polylines](../../d6/d6e/group__imgproc__draw.html#ga1ea127ffbbb7e0bfc4fd6fd2eb64263c "../../d6/d6e/group__imgproc__draw.html#ga1ea127ffbbb7e0bfc4fd6fd2eb64263c")(img2,[np.int32(dst)],True,255,3, cv.LINE\_AA)else: [print](../../df/d57/namespacecv_1_1dnn.html#a43417dcaeb3c1e2a09b9d948e234c366 "../../df/d57/namespacecv_1_1dnn.html#a43417dcaeb3c1e2a09b9d948e234c366")( "Not enough matches are found - {}/{}".[format](../../db/de0/group__core__utils.html#ga0cccdb2f73859309b0611cf70b1b9409 "../../db/de0/group__core__utils.html#ga0cccdb2f73859309b0611cf70b1b9409")(len(good), MIN\_MATCH\_COUNT) ) matchesMask = None Finally we draw our inliers (if successfully found the object) or matching keypoints (if failed). 

draw\_params = dict(matchColor = (0,255,0), # draw matches in green color singlePointColor = None, matchesMask = matchesMask, # draw only inliers flags = 2)img3 = [cv.drawMatches](../../d4/d5d/group__features2d__draw.html#ga62fbedb5206ab2faf411797e7055c90f "../../d4/d5d/group__features2d__draw.html#ga62fbedb5206ab2faf411797e7055c90f")(img1,kp1,img2,kp2,good,None,\*\*draw\_params)plt.imshow(img3, 'gray'),plt.show() See the result below. Object is marked in white color in cluttered image:

![homography_findobj.jpg](../../homography_findobj.jpg)

image
## Additional Resources

## Exercises

---

Generated on Wed Oct 4 2023 23:37:11 for OpenCV by  [![doxygen](../../doxygen.png)](http://www.doxygen.org/index.html "http://www.doxygen.org/index.html") 1.8.13

//<![CDATA[
addTutorialsButtons();
//]]>

