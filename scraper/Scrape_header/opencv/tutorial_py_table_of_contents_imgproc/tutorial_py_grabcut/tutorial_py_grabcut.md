

OpenCV: Interactive Foreground Extraction using GrabCut Algorithm

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

Interactive Foreground Extraction using GrabCut Algorithm  

## Goal

In this chapter

* We will see GrabCut algorithm to extract foreground in images
* We will create an interactive application for this.

## Theory

GrabCut algorithm was designed by Carsten Rother, Vladimir Kolmogorov & Andrew Blake from Microsoft Research Cambridge, UK. in their paper, ["GrabCut": interactive foreground extraction using iterated graph cuts](http://dl.acm.org/citation.cfm?id=1015720 "http://dl.acm.org/citation.cfm?id=1015720") . An algorithm was needed for foreground extraction with minimal user interaction, and the result was GrabCut.

How it works from user point of view ? Initially user draws a rectangle around the foreground region (foreground region should be completely inside the rectangle). Then algorithm segments it iteratively to get the best result. Done. But in some cases, the segmentation won't be fine, like, it may have marked some foreground region as background and vice versa. In that case, user need to do fine touch-ups. Just give some strokes on the images where some faulty results are there. Strokes basically says \*"Hey, this region should be foreground, you marked it background, correct it in next
iteration"\* or its opposite for background. Then in the next iteration, you get better results.

See the image below. First player and football is enclosed in a blue rectangle. Then some final touchups with white strokes (denoting foreground) and black strokes (denoting background) is made. And we get a nice result.

![grabcut_output1.jpg](../../grabcut_output1.jpg)

image
 So what happens in background ?

* User inputs the rectangle. Everything outside this rectangle will be taken as sure background (That is the reason it is mentioned before that your rectangle should include all the objects). Everything inside rectangle is unknown. Similarly any user input specifying foreground and background are considered as hard-labelling which means they won't change in the process.
* Computer does an initial labelling depending on the data we gave. It labels the foreground and background pixels (or it hard-labels)
* Now a Gaussian Mixture Model(GMM) is used to model the foreground and background.
* Depending on the data we gave, GMM learns and create new pixel distribution. That is, the unknown pixels are labelled either probable foreground or probable background depending on its relation with the other hard-labelled pixels in terms of color statistics (It is just like clustering).
* A graph is built from this pixel distribution. Nodes in the graphs are pixels. Additional two nodes are added, **Source node** and **Sink node**. Every foreground pixel is connected to Source node and every background pixel is connected to Sink node.
* The weights of edges connecting pixels to source node/end node are defined by the probability of a pixel being foreground/background. The weights between the pixels are defined by the edge information or pixel similarity. If there is a large difference in pixel color, the edge between them will get a low weight.
* Then a mincut algorithm is used to segment the graph. It cuts the graph into two separating source node and sink node with minimum cost function. The cost function is the sum of all weights of the edges that are cut. After the cut, all the pixels connected to Source node become foreground and those connected to Sink node become background.
* The process is continued until the classification converges.

It is illustrated in below image (Image Courtesy: [http://www.cs.ru.ac.za/research/g02m1682/](http://www.cs.ru.ac.za/research/g02m1682/ "http://www.cs.ru.ac.za/research/g02m1682/"))

![grabcut_scheme.jpg](../../grabcut_scheme.jpg)

image
## Demo

Now we go for grabcut algorithm with OpenCV. OpenCV has the function, **[cv.grabCut()](../../d3/d47/group__imgproc__segmentation.html#ga909c1dda50efcbeaa3ce126be862b37f "Runs the GrabCut algorithm. ")** for this. We will see its arguments first:

* *img* - Input image
* *mask* - It is a mask image where we specify which areas are background, foreground or probable background/foreground etc. It is done by the following flags, **[cv.GC\_BGD](../../d7/d1b/group__imgproc__misc.html#ggad43d3e4208d3cf025d8304156b02ba38a889f1ce109543e8aed80a7abbc6dcb39 "an obvious background pixels "), [cv.GC\_FGD](../../d7/d1b/group__imgproc__misc.html#ggad43d3e4208d3cf025d8304156b02ba38a4757c1f0587bcf6e53e86dee7689a649 "an obvious foreground (object) pixel "), [cv.GC\_PR\_BGD](../../d7/d1b/group__imgproc__misc.html#ggad43d3e4208d3cf025d8304156b02ba38af748414821c7f39fab3493f9eed1eedf "a possible background pixel "), [cv.GC\_PR\_FGD](../../d7/d1b/group__imgproc__misc.html#ggad43d3e4208d3cf025d8304156b02ba38ad33184b73cb87e08d29e0a3411b7c863 "a possible foreground pixel ")**, or simply pass 0,1,2,3 to image.
* *rect* - It is the coordinates of a rectangle which includes the foreground object in the format (x,y,w,h)
* *bdgModel*, *fgdModel* - These are arrays used by the algorithm internally. You just create two np.float64 type zero arrays of size (1,65).
* *iterCount* - Number of iterations the algorithm should run.
* *mode* - It should be **[cv.GC\_INIT\_WITH\_RECT](../../d7/d1b/group__imgproc__misc.html#ggaf8b5832ba85e59fc7a98a2afd034e558a5f8853c1e5a89c4aa2687d1f78a7e550 "../../d7/d1b/group__imgproc__misc.html#ggaf8b5832ba85e59fc7a98a2afd034e558a5f8853c1e5a89c4aa2687d1f78a7e550")** or **[cv.GC\_INIT\_WITH\_MASK](../../d7/d1b/group__imgproc__misc.html#ggaf8b5832ba85e59fc7a98a2afd034e558ab01527c7effb50fd1c54d8c4e671ed22 "../../d7/d1b/group__imgproc__misc.html#ggaf8b5832ba85e59fc7a98a2afd034e558ab01527c7effb50fd1c54d8c4e671ed22")** or combined which decides whether we are drawing rectangle or final touchup strokes.

First let's see with rectangular mode. We load the image, create a similar mask image. We create *fgdModel* and *bgdModel*. We give the rectangle parameters. It's all straight-forward. Let the algorithm run for 5 iterations. Mode should be *[cv.GC\_INIT\_WITH\_RECT](../../d7/d1b/group__imgproc__misc.html#ggaf8b5832ba85e59fc7a98a2afd034e558a5f8853c1e5a89c4aa2687d1f78a7e550 "../../d7/d1b/group__imgproc__misc.html#ggaf8b5832ba85e59fc7a98a2afd034e558a5f8853c1e5a89c4aa2687d1f78a7e550")* since we are using rectangle. Then run the grabcut. It modifies the mask image. In the new mask image, pixels will be marked with four flags denoting background/foreground as specified above. So we modify the mask such that all 0-pixels and 2-pixels are put to 0 (ie background) and all 1-pixels and 3-pixels are put to 1(ie foreground pixels). Now our final mask is ready. Just multiply it with input image to get the segmented image. 

import numpy as npimport cv2 as cvfrom matplotlib import pyplot as pltimg = [cv.imread](../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56 "../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56")('messi5.jpg')assert img is not None, "file could not be read, check with os.path.exists()"mask = np.zeros(img.shape[:2],np.uint8)bgdModel = np.zeros((1,65),np.float64)fgdModel = np.zeros((1,65),np.float64)rect = (50,50,450,290)[cv.grabCut](../../d3/d47/group__imgproc__segmentation.html#ga909c1dda50efcbeaa3ce126be862b37f "../../d3/d47/group__imgproc__segmentation.html#ga909c1dda50efcbeaa3ce126be862b37f")(img,mask,rect,bgdModel,fgdModel,5,cv.GC\_INIT\_WITH\_RECT)mask2 = np.where((mask==2)|(mask==0),0,1).astype('uint8')img = img\*mask2[:,:,np.newaxis]plt.imshow(img),plt.colorbar(),plt.show() See the results below:

![grabcut_rect.jpg](../../grabcut_rect.jpg)

image
 Oops, Messi's hair is gone. *Who likes Messi without his hair?* We need to bring it back. So we will give there a fine touchup with 1-pixel (sure foreground). At the same time, Some part of ground has come to picture which we don't want, and also some logo. We need to remove them. There we give some 0-pixel touchup (sure background). So we modify our resulting mask in previous case as we told now.

*What I actually did is that, I opened input image in paint application and added another layer to the image. Using brush tool in the paint, I marked missed foreground (hair, shoes, ball etc) with white and unwanted background (like logo, ground etc) with black on this new layer. Then filled remaining background with gray. Then loaded that mask image in OpenCV, edited original mask image we got with corresponding values in newly added mask image. Check the code below:* 

# newmask is the mask image I manually labellednewmask = [cv.imread](../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56 "../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56")('newmask.png', cv.IMREAD\_GRAYSCALE)assert newmask is not None, "file could not be read, check with os.path.exists()"# wherever it is marked white (sure foreground), change mask=1# wherever it is marked black (sure background), change mask=0mask[newmask == 0] = 0mask[newmask == 255] = 1mask, bgdModel, fgdModel = [cv.grabCut](../../d3/d47/group__imgproc__segmentation.html#ga909c1dda50efcbeaa3ce126be862b37f "../../d3/d47/group__imgproc__segmentation.html#ga909c1dda50efcbeaa3ce126be862b37f")(img,mask,None,bgdModel,fgdModel,5,cv.GC\_INIT\_WITH\_MASK)mask = np.where((mask==2)|(mask==0),0,1).astype('uint8')img = img\*mask[:,:,np.newaxis]plt.imshow(img),plt.colorbar(),plt.show() See the result below:

![grabcut_mask.jpg](../../grabcut_mask.jpg)

image
 So that's it. Here instead of initializing in rect mode, you can directly go into mask mode. Just mark the rectangle area in mask image with 2-pixel or 3-pixel (probable background/foreground). Then mark our sure\_foreground with 1-pixel as we did in second example. Then directly apply the grabCut function with mask mode.

## Additional Resources

## Exercises

1. OpenCV samples contain a sample grabcut.py which is an interactive tool using grabcut. Check it. Also watch this [youtube video](https://www.youtube.com/watch?v=kAwxLTDDAwU "https://www.youtube.com/watch?v=kAwxLTDDAwU") on how to use it.
2. Here, you can make this into a interactive sample with drawing rectangle and strokes with mouse, create trackbar to adjust stroke width etc.

---

Generated on Wed Oct 4 2023 23:37:11 for OpenCV by  [![doxygen](../../doxygen.png)](http://www.doxygen.org/index.html "http://www.doxygen.org/index.html") 1.8.13

//<![CDATA[
addTutorialsButtons();
//]]>

