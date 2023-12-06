

OpenCV: Install OpenCV-Python in Fedora

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
* [Introduction to OpenCV](../../da/df6/tutorial_py_table_of_contents_setup.html "../../da/df6/tutorial_py_table_of_contents_setup.html")

Install OpenCV-Python in Fedora  

## Goals

In this tutorial

* We will learn to setup OpenCV-Python in your Fedora system. Below steps are tested for Fedora 18 (64-bit) and Fedora 19 (32-bit).

## Introduction

OpenCV-Python can be installed in Fedora in two ways, 1) Install from pre-built binaries available in fedora repositories, 2) Compile from the source. In this section, we will see both.

Another important thing is the additional libraries required. OpenCV-Python requires only **Numpy** (in addition to other dependencies, which we will see later). But in this tutorials, we also use **Matplotlib** for some easy and nice plotting purposes (which I feel much better compared to OpenCV). Matplotlib is optional, but highly recommended. Similarly we will also see **IPython**, an Interactive Python Terminal, which is also highly recommended.

## Installing OpenCV-Python from Pre-built Binaries

Install all packages with following command in terminal as root. 

$ yum install numpy opencv\* Open Python IDLE (or IPython) and type following codes in Python terminal. 

>>> import cv2 as cv>>> [print](../../df/d57/namespacecv_1_1dnn.html#a43417dcaeb3c1e2a09b9d948e234c366 "../../df/d57/namespacecv_1_1dnn.html#a43417dcaeb3c1e2a09b9d948e234c366")( cv.\_\_version\_\_ ) If the results are printed out without any errors, congratulations !!! You have installed OpenCV-Python successfully.

It is quite easy. But there is a problem with this. Yum repositories may not contain the latest version of OpenCV always. For example, at the time of writing this tutorial, yum repository contains 2.4.5 while latest OpenCV version is 2.4.6. With respect to Python API, latest version will always contain much better support. Also, there may be chance of problems with camera support, video playback etc depending upon the drivers, ffmpeg, gstreamer packages present etc.

So my personal preference is next method, i.e. compiling from source. Also at some point in time, if you want to contribute to OpenCV, you will need this.

## Installing OpenCV from source

Compiling from source may seem a little complicated at first, but once you succeeded in it, there is nothing complicated.

First we will install some dependencies. Some are compulsory, some are optional. Optional dependencies, you can leave if you don't want.

### Compulsory Dependencies

We need **CMake** to configure the installation, **GCC** for compilation, **Python-devel** and **Numpy** for creating Python extensions etc. 

yum install cmakeyum install python-devel numpyyum install gcc gcc-c++ Next we need **GTK** support for GUI features, Camera support (libdc1394, v4l), Media Support (ffmpeg, gstreamer) etc. 

yum install gtk2-develyum install libdc1394-develyum install ffmpeg-develyum install gstreamer-plugins-base-devel ### Optional Dependencies

Above dependencies are sufficient to install OpenCV in your fedora machine. But depending upon your requirements, you may need some extra dependencies. A list of such optional dependencies are given below. You can either leave it or install it, your call :)

OpenCV comes with supporting files for image formats like PNG, JPEG, JPEG2000, TIFF, WebP etc. But it may be a little old. If you want to get latest libraries, you can install development files for these formats. 

yum install libpng-develyum install libjpeg-turbo-develyum install jasper-develyum install openexr-develyum install libtiff-develyum install libwebp-devel Several OpenCV functions are parallelized with **Intel's Threading Building Blocks** (TBB). But if you want to enable it, you need to install TBB first. ( Also while configuring installation with CMake, don't forget to pass -D WITH\_TBB=ON. More details below.) 

yum install tbb-devel OpenCV uses another library **Eigen** for optimized mathematical operations. So if you have Eigen installed in your system, you can exploit it. ( Also while configuring installation with CMake, don't forget to pass -D WITH\_EIGEN=ON. More details below.) 

yum install eigen3-devel If you want to build **documentation** ( *Yes, you can create offline version of OpenCV's complete official documentation in your system in HTML with full search facility so that you need not access internet always if any question, and it is quite FAST!!!* ), you need to install **Doxygen** (a documentation generation tool). 

yum install doxygen ### Downloading OpenCV

Next we have to download OpenCV. You can download the latest release of OpenCV from [sourceforge site](http://sourceforge.net/projects/opencvlibrary/ "http://sourceforge.net/projects/opencvlibrary/"). Then extract the folder.

Or you can download latest source from OpenCV's github repo. (If you want to contribute to OpenCV, choose this. It always keeps your OpenCV up-to-date). For that, you need to install **Git** first. 

yum install gitgit clone https://github.com/opencv/opencv.git It will create a folder OpenCV in home directory (or the directory you specify). The cloning may take some time depending upon your internet connection.

Now open a terminal window and navigate to the downloaded OpenCV folder. Create a new build folder and navigate to it. 

mkdir buildcd build ### Configuring and Installing

Now we have installed all the required dependencies, let's install OpenCV. Installation has to be configured with CMake. It specifies which modules are to be installed, installation path, which additional libraries to be used, whether documentation and examples to be compiled etc. Below command is normally used for configuration (executed from build folder). 

cmake -D CMAKE\_BUILD\_TYPE=RELEASE -D CMAKE\_INSTALL\_PREFIX=/usr/local .. It specifies that build type is "Release Mode" and installation path is /usr/local. Observe the -D before each option and .. at the end. In short, this is the format: 

cmake [-D <flag>] [-D <flag>] .. You can specify as many flags you want, but each flag should be preceded by -D.

So in this tutorial, we are installing OpenCV with TBB and Eigen support. We also build the documentation, but we exclude Performance tests and building samples. We also disable GPU related modules (since we use OpenCV-Python, we don't need GPU related modules. It saves us some time).

\*(All the below commands can be done in a single cmake statement, but it is split here for better understanding.)\*

* Enable TBB and Eigen support: cmake -D WITH\_TBB=ON -D WITH\_EIGEN=ON ..
* Enable documentation and disable tests and samples cmake -D BUILD\_DOCS=ON -D BUILD\_TESTS=OFF -D BUILD\_PERF\_TESTS=OFF -D BUILD\_EXAMPLES=OFF ..
* Disable all GPU related modules. cmake -D WITH\_OPENCL=OFF -D BUILD\_opencv\_gpu=OFF -D BUILD\_opencv\_gpuarithm=OFF -D BUILD\_opencv\_gpubgsegm=OFF -D BUILD\_opencv\_gpucodec=OFF -D BUILD\_opencv\_gpufeatures2d=OFF -D BUILD\_opencv\_gpufilters=OFF -D BUILD\_opencv\_gpuimgproc=OFF -D BUILD\_opencv\_gpulegacy=OFF -D BUILD\_opencv\_gpuoptflow=OFF -D BUILD\_opencv\_gpustereo=OFF -D BUILD\_opencv\_gpuwarping=OFF ..
* Set installation path and build type cmake -D CMAKE\_BUILD\_TYPE=RELEASE -D CMAKE\_INSTALL\_PREFIX=/usr/local .. Each time you enter cmake statement, it prints out the resulting configuration setup. In the final setup you got, make sure that following fields are filled (below is the some important parts of configuration I got). These fields should be filled appropriately in your system also. Otherwise some problem has happened. So check if you have correctly performed above steps. ...-- GUI:-- GTK+ 2.x: YES (ver 2.24.19)-- GThread : YES (ver 2.36.3)-- Video I/O:-- DC1394 2.x: YES (ver 2.2.0)-- FFMPEG: YES-- codec: YES (ver 54.92.100)-- format: YES (ver 54.63.104)-- util: YES (ver 52.18.100)-- swscale: YES (ver 2.2.100)-- gentoo-style: YES-- GStreamer:-- base: YES (ver 0.10.36)-- video: YES (ver 0.10.36)-- app: YES (ver 0.10.36)-- riff: YES (ver 0.10.36)-- pbutils: YES (ver 0.10.36)-- V4L/V4L2: Using libv4l (ver 1.0.0)-- Other third-party libraries:-- Use Eigen: YES (ver 3.1.4)-- Use TBB: YES (ver 4.0 interface 6004)-- Python:-- Interpreter: /usr/bin/python2 (ver 2.7.5)-- Libraries: /lib/libpython2.7.so (ver 2.7.5)-- numpy: /usr/lib/python2.7/site-packages/numpy/core/include (ver 1.7.1)-- packages path: lib/python2.7/site-packages... Many other flags and settings are there. It is left for you for further exploration.

Now you build the files using make command and install it using make install command. make install should be executed as root. 

makesumake install Installation is over. All files are installed in /usr/local/ folder. But to use it, your Python should be able to find OpenCV module. You have two options for that.

1. **Move the module to any folder in Python Path** : Python path can be found out by entering `import sys; print(sys.path)` in Python terminal. It will print out many locations. Move /usr/local/lib/python2.7/site-packages/cv2.so to any of this folder. For example, su mv /usr/local/lib/python2.7/site-packages/cv2.so /usr/lib/python2.7/site-packages But you will have to do this every time you install OpenCV.
2. **Add /usr/local/lib/python2.7/site-packages to the PYTHON\_PATH**: It is to be done only once. Just open/.bashrc and add following line to it, then log out and come back. export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python2.7/site-packages Thus OpenCV installation is finished. Open a terminal and try 'import cv2 as cv'.

To build the documentation, just enter following commands: 

make doxygen Then open opencv/build/doc/doxygen/html/index.html and bookmark it in the browser.

## Additional Resources

## Exercises

1. Compile OpenCV from source in your Fedora machine.

---

Generated on Wed Oct 4 2023 23:37:11 for OpenCV by  [![doxygen](../../doxygen.png)](http://www.doxygen.org/index.html "http://www.doxygen.org/index.html") 1.8.13

//<![CDATA[
addTutorialsButtons();
//]]>

