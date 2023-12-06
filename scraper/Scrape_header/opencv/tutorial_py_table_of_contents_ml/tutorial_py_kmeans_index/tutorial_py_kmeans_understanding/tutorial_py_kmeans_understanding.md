

OpenCV: Understanding K-Means Clustering

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
* [Machine Learning](../../d6/de2/tutorial_py_table_of_contents_ml.html "../../d6/de2/tutorial_py_table_of_contents_ml.html")
* [K-Means Clustering](../../d9/d70/tutorial_py_kmeans_index.html "../../d9/d70/tutorial_py_kmeans_index.html")

Understanding K-Means Clustering  

## Goal

In this chapter, we will understand the concepts of K-Means Clustering, how it works etc.

## Theory

We will deal this with an example which is commonly used.

### T-shirt size problem

Consider a company, which is going to release a new model of T-shirt to market. Obviously they will have to manufacture models in different sizes to satisfy people of all sizes. So the company make a data of people's height and weight, and plot them on to a graph, as below:

![tshirt.jpg](../../tshirt.jpg)

image
 Company can't create t-shirts with all the sizes. Instead, they divide people to Small, Medium and Large, and manufacture only these 3 models which will fit into all the people. This grouping of people into three groups can be done by k-means clustering, and algorithm provides us best 3 sizes, which will satisfy all the people. And if it doesn't, company can divide people to more groups, may be five, and so on. Check image below :

![tshirt_grouped.jpg](../../tshirt_grouped.jpg)

image
### How does it work ?

This algorithm is an iterative process. We will explain it step-by-step with the help of images.

Consider a set of data as below ( You can consider it as t-shirt problem). We need to cluster this data into two groups.

![testdata.jpg](../../testdata.jpg)

image
 **Step : 1** - Algorithm randomly chooses two centroids, \(C1\) and \(C2\) (sometimes, any two data are taken as the centroids).

**Step : 2** - It calculates the distance from each point to both centroids. If a test data is more closer to \(C1\), then that data is labelled with '0'. If it is closer to \(C2\), then labelled as '1' (If more centroids are there, labelled as '2','3' etc).

In our case, we will color all '0' labelled with red, and '1' labelled with blue. So we get following image after above operations.

![initial_labelling.jpg](../../initial_labelling.jpg)

image
 **Step : 3** - Next we calculate the average of all blue points and red points separately and that will be our new centroids. That is \(C1\) and \(C2\) shift to newly calculated centroids. (Remember, the images shown are not true values and not to true scale, it is just for demonstration only).

And again, perform step 2 with new centroids and label data to '0' and '1'.

So we get result as below :

![update_centroid.jpg](../../update_centroid.jpg)

image
 Now **Step - 2** and **Step - 3** are iterated until both centroids are converged to fixed points. \*(Or it may be stopped depending on the criteria we provide, like maximum number of iterations, or a specific accuracy is reached etc.)\* **These points are such that sum of distances between test data and their corresponding centroids are minimum**. Or simply, sum of distances between \(C1 \leftrightarrow Red\\_Points\) and \(C2 \leftrightarrow Blue\\_Points\) is minimum.

\[minimize \;\bigg[J = \sum\_{All\: Red\\_Points}distance(C1,Red\\_Point) + \sum\_{All\: Blue\\_Points}distance(C2,Blue\\_Point)\bigg]\]

Final result almost looks like below :

![final_clusters.jpg](../../final_clusters.jpg)

image
 So this is just an intuitive understanding of K-Means Clustering. For more details and mathematical explanation, please read any standard machine learning textbooks or check links in additional resources. It is just a top layer of K-Means clustering. There are a lot of modifications to this algorithm like, how to choose the initial centroids, how to speed up the iteration process etc.

## Additional Resources

1. [Machine Learning Course](https://www.coursera.org/course/ml "https://www.coursera.org/course/ml"), Video lectures by Prof. Andrew Ng (Some of the images are taken from this)

## Exercises

---

Generated on Wed Oct 4 2023 23:37:11 for OpenCV by  [![doxygen](../../doxygen.png)](http://www.doxygen.org/index.html "http://www.doxygen.org/index.html") 1.8.13

//<![CDATA[
addTutorialsButtons();
//]]>

