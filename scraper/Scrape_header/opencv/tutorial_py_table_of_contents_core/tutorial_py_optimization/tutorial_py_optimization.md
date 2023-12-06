

OpenCV: Performance Measurement and Improvement Techniques

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

Performance Measurement and Improvement Techniques  

## Goal

In image processing, since you are dealing with a large number of operations per second, it is mandatory that your code is not only providing the correct solution, but that it is also providing it in the fastest manner. So in this chapter, you will learn:

* To measure the performance of your code.
* Some tips to improve the performance of your code.
* You will see these functions: **[cv.getTickCount](../../db/de0/group__core__utils.html#gae73f58000611a1af25dd36d496bf4487 "Returns the number of ticks. ")**, **[cv.getTickFrequency](../../db/de0/group__core__utils.html#ga705441a9ef01f47acdc55d87fbe5090c "Returns the number of ticks per second. ")**, etc.

Apart from OpenCV, Python also provides a module **time** which is helpful in measuring the time of execution. Another module **profile** helps to get a detailed report on the code, like how much time each function in the code took, how many times the function was called, etc. But, if you are using IPython, all these features are integrated in a user-friendly manner. We will see some important ones, and for more details, check links in the **Additional Resources** section.

## Measuring Performance with OpenCV

The **[cv.getTickCount](../../db/de0/group__core__utils.html#gae73f58000611a1af25dd36d496bf4487 "Returns the number of ticks. ")** function returns the number of clock-cycles after a reference event (like the moment the machine was switched ON) to the moment this function is called. So if you call it before and after the function execution, you get the number of clock-cycles used to execute a function.

The **[cv.getTickFrequency](../../db/de0/group__core__utils.html#ga705441a9ef01f47acdc55d87fbe5090c "Returns the number of ticks per second. ")** function returns the frequency of clock-cycles, or the number of clock-cycles per second. So to find the time of execution in seconds, you can do following: 

e1 = [cv.getTickCount](../../db/de0/group__core__utils.html#gae73f58000611a1af25dd36d496bf4487 "../../db/de0/group__core__utils.html#gae73f58000611a1af25dd36d496bf4487")()# your code executione2 = [cv.getTickCount](../../db/de0/group__core__utils.html#gae73f58000611a1af25dd36d496bf4487 "../../db/de0/group__core__utils.html#gae73f58000611a1af25dd36d496bf4487")()time = (e2 - e1)/ [cv.getTickFrequency](../../db/de0/group__core__utils.html#ga705441a9ef01f47acdc55d87fbe5090c "../../db/de0/group__core__utils.html#ga705441a9ef01f47acdc55d87fbe5090c")() We will demonstrate with following example. The following example applies median filtering with kernels of odd sizes ranging from 5 to 49. (Don't worry about what the result will look like - that is not our goal): 

img1 = [cv.imread](../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56 "../../d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56")('messi5.jpg')assert img1 is not None, "file could not be read, check with os.path.exists()"e1 = [cv.getTickCount](../../db/de0/group__core__utils.html#gae73f58000611a1af25dd36d496bf4487 "../../db/de0/group__core__utils.html#gae73f58000611a1af25dd36d496bf4487")()for i in range(5,49,2): img1 = [cv.medianBlur](../../d4/d86/group__imgproc__filter.html#ga564869aa33e58769b4469101aac458f9 "../../d4/d86/group__imgproc__filter.html#ga564869aa33e58769b4469101aac458f9")(img1,i)e2 = [cv.getTickCount](../../db/de0/group__core__utils.html#gae73f58000611a1af25dd36d496bf4487 "../../db/de0/group__core__utils.html#gae73f58000611a1af25dd36d496bf4487")()t = (e2 - e1)/[cv.getTickFrequency](../../db/de0/group__core__utils.html#ga705441a9ef01f47acdc55d87fbe5090c "../../db/de0/group__core__utils.html#ga705441a9ef01f47acdc55d87fbe5090c")()[print](../../df/d57/namespacecv_1_1dnn.html#a43417dcaeb3c1e2a09b9d948e234c366 "../../df/d57/namespacecv_1_1dnn.html#a43417dcaeb3c1e2a09b9d948e234c366")( t )# Result I got is 0.521107655 seconds NoteYou can do the same thing with the time module. Instead of [cv.getTickCount](../../db/de0/group__core__utils.html#gae73f58000611a1af25dd36d496bf4487 "Returns the number of ticks. "), use the time.time() function. Then take the difference of the two times.
## Default Optimization in OpenCV

Many of the OpenCV functions are optimized using SSE2, AVX, etc. It contains the unoptimized code also. So if our system support these features, we should exploit them (almost all modern day processors support them). It is enabled by default while compiling. So OpenCV runs the optimized code if it is enabled, otherwise it runs the unoptimized code. You can use **[cv.useOptimized()](../../db/de0/group__core__utils.html#gafa6d5d04eff341825573ec6c0aa6519f "Returns the status of optimized code usage. ")** to check if it is enabled/disabled and **[cv.setUseOptimized()](../../db/de0/group__core__utils.html#ga3c8487ea4449e550bc39575ede094c7a "Enables or disables the optimized code. ")** to enable/disable it. Let's see a simple example. 

# check if optimization is enabledIn [5]: [cv.useOptimized](../../db/de0/group__core__utils.html#gafa6d5d04eff341825573ec6c0aa6519f "../../db/de0/group__core__utils.html#gafa6d5d04eff341825573ec6c0aa6519f")()Out[5]: TrueIn [6]: %timeit res = [cv.medianBlur](../../d4/d86/group__imgproc__filter.html#ga564869aa33e58769b4469101aac458f9 "../../d4/d86/group__imgproc__filter.html#ga564869aa33e58769b4469101aac458f9")(img,49)10 loops, best of 3: 34.9 ms per loop# Disable itIn [7]: [cv.setUseOptimized](../../db/de0/group__core__utils.html#ga3c8487ea4449e550bc39575ede094c7a "../../db/de0/group__core__utils.html#ga3c8487ea4449e550bc39575ede094c7a")(False)In [8]: [cv.useOptimized](../../db/de0/group__core__utils.html#gafa6d5d04eff341825573ec6c0aa6519f "../../db/de0/group__core__utils.html#gafa6d5d04eff341825573ec6c0aa6519f")()Out[8]: FalseIn [9]: %timeit res = [cv.medianBlur](../../d4/d86/group__imgproc__filter.html#ga564869aa33e58769b4469101aac458f9 "../../d4/d86/group__imgproc__filter.html#ga564869aa33e58769b4469101aac458f9")(img,49)10 loops, best of 3: 64.1 ms per loop As you can see, optimized median filtering is2x faster than the unoptimized version. If you check its source, you can see that median filtering is SIMD optimized. So you can use this to enable optimization at the top of your code (remember it is enabled by default).

## Measuring Performance in IPython

Sometimes you may need to compare the performance of two similar operations. IPython gives you a magic command timeit to perform this. It runs the code several times to get more accurate results. Once again, it is suitable to measuring single lines of code.

For example, do you know which of the following addition operations is better, x = 5; y = x\*\*2, x = 5; y = x\*x, x = np.uint8([5]); y = x\*x, or y = np.square(x)? We will find out with timeit in the IPython shell. 

In [10]: x = 5In [11]: %timeit y=x\*\*210000000 loops, best of 3: 73 ns per loopIn [12]: %timeit y=x\*x10000000 loops, best of 3: 58.3 ns per loopIn [15]: z = np.uint8([5])In [17]: %timeit y=z\*z1000000 loops, best of 3: 1.25 us per loopIn [19]: %timeit y=np.square(z)1000000 loops, best of 3: 1.16 us per loop You can see that, x = 5 ; y = x\*x is fastest and it is around 20x faster compared to Numpy. If you consider the array creation also, it may reach up to 100x faster. Cool, right? \*(Numpy devs are working on this issue)\*

NotePython scalar operations are faster than Numpy scalar operations. So for operations including one or two elements, Python scalar is better than Numpy arrays. Numpy has the advantage when the size of the array is a little bit bigger.
We will try one more example. This time, we will compare the performance of **[cv.countNonZero()](../../d2/de8/group__core__array.html#gaa4b89393263bb4d604e0fe5986723914 "Counts non-zero array elements. ")** and **np.count\_nonzero()** for the same image.

In [35]: %timeit z = [cv.countNonZero](../../d2/de8/group__core__array.html#gaa4b89393263bb4d604e0fe5986723914 "../../d2/de8/group__core__array.html#gaa4b89393263bb4d604e0fe5986723914")(img)100000 loops, best of 3: 15.8 us per loopIn [36]: %timeit z = np.count\_nonzero(img)1000 loops, best of 3: 370 us per loop See, the OpenCV function is nearly 25x faster than the Numpy function.

NoteNormally, OpenCV functions are faster than Numpy functions. So for same operation, OpenCV functions are preferred. But, there can be exceptions, especially when Numpy works with views instead of copies.
## More IPython magic commands

There are several other magic commands to measure performance, profiling, line profiling, memory measurement, and etc. They all are well documented. So only links to those docs are provided here. Interested readers are recommended to try them out.

## Performance Optimization Techniques

There are several techniques and coding methods to exploit maximum performance of Python and Numpy. Only relevant ones are noted here and links are given to important sources. The main thing to be noted here is, first try to implement the algorithm in a simple manner. Once it is working, profile it, find the bottlenecks, and optimize them.

1. Avoid using loops in Python as much as possible, especially double/triple loops etc. They are inherently slow.
2. Vectorize the algorithm/code to the maximum extent possible, because Numpy and OpenCV are optimized for vector operations.
3. Exploit the cache coherence.
4. Never make copies of an array unless it is necessary. Try to use views instead. Array copying is a costly operation.

If your code is still slow after doing all of these operations, or if the use of large loops is inevitable, use additional libraries like Cython to make it faster.

## Additional Resources

1. [Python Optimization Techniques](https://wiki.python.org/moin/PythonSpeed/PerformanceTips "https://wiki.python.org/moin/PythonSpeed/PerformanceTips")
2. Scipy Lecture Notes - [Advanced Numpy](http://scipy-lectures.github.io/advanced/advanced_numpy/index.html#advanced-numpy "http://scipy-lectures.github.io/advanced/advanced_numpy/index.html#advanced-numpy")
3. [Timing and Profiling in IPython](http://pynash.org/2013/03/06/timing-and-profiling/ "http://pynash.org/2013/03/06/timing-and-profiling/")

## Exercises

---

Generated on Wed Oct 4 2023 23:37:11 for OpenCV by  [![doxygen](../../doxygen.png)](http://www.doxygen.org/index.html "http://www.doxygen.org/index.html") 1.8.13

//<![CDATA[
addTutorialsButtons();
//]]>

