
# Goal

In this chapter

* We will revisit the hand-written data OCR, but, with SVM instead of kNN.

# OCR of Hand-written Digits

In kNN, we directly used pixel intensity as the feature vector. This time we will use [Histogram of Oriented Gradients](https://en.wikipedia.org/wiki/Histogram_of_oriented_gradients "https://en.wikipedia.org/wiki/Histogram_of_oriented_gradients") (HOG) as feature vectors.

Here, before finding the HOG, we deskew the image using its second order moments. So we first define a function **deskew()** which takes a digit image and deskew it. Below is the deskew() function:

def deskew(img):
 m = [cv.moments](../../d3/dc0/group__imgproc__shape.html#ga556a180f43cab22649c23ada36a8a139 "../../d3/dc0/group__imgproc__shape.html#ga556a180f43cab22649c23ada36a8a139")(img)
 if abs(m['mu02']) < 1e-2:
 return img.copy()
 skew = m['mu11']/m['mu02']
 M = np.float32([[1, skew, -0.5\*SZ\*skew], [0, 1, 0]])
 img = [cv.warpAffine](../../da/d54/group__imgproc__transform.html#ga0203d9ee5fcd28d40dbc4a1ea4451983 "../../da/d54/group__imgproc__transform.html#ga0203d9ee5fcd28d40dbc4a1ea4451983")(img,M,(SZ, SZ),flags=affine\_flags)
 return img
[cv::moments](../../d3/dc0/group__imgproc__shape.html#ga556a180f43cab22649c23ada36a8a139 "../../d3/dc0/group__imgproc__shape.html#ga556a180f43cab22649c23ada36a8a139")Moments moments(InputArray array, bool binaryImage=false)Calculates all of the moments up to the third order of a polygon or rasterized shape.
[cv::warpAffine](../../da/d54/group__imgproc__transform.html#ga0203d9ee5fcd28d40dbc4a1ea4451983 "../../da/d54/group__imgproc__transform.html#ga0203d9ee5fcd28d40dbc4a1ea4451983")void warpAffine(InputArray src, OutputArray dst, InputArray M, Size dsize, int flags=INTER\_LINEAR, int borderMode=BORDER\_CONSTANT, const Scalar &borderValue=Scalar())Applies an affine transformation to an image.
Below image shows above deskew function applied to an image of zero. Left image is the original image and right image is the deskewed image.

![](../../deskew.jpg)

image
Next we have to find the HOG Descriptor of each cell. For that, we find Sobel derivatives of each cell in X and Y direction. Then find their magnitude and direction of gradient at each pixel. This gradient is quantized to 16 integer values. Divide this image to four sub-squares. For each sub-square, calculate the histogram of direction (16 bins) weighted with their magnitude. So each sub-square gives you a vector containing 16 values. Four such vectors (of four sub-squares) together gives us a feature vector containing 64 values. This is the feature vector we use to train our data.

def hog(img):
 gx = [cv.Sobel](../../d4/d86/group__imgproc__filter.html#gacea54f142e81b6758cb6f375ce782c8d "../../d4/d86/group__imgproc__filter.html#gacea54f142e81b6758cb6f375ce782c8d")(img, cv.CV\_32F, 1, 0)
 gy = [cv.Sobel](../../d4/d86/group__imgproc__filter.html#gacea54f142e81b6758cb6f375ce782c8d "../../d4/d86/group__imgproc__filter.html#gacea54f142e81b6758cb6f375ce782c8d")(img, cv.CV\_32F, 0, 1)
 mag, ang = [cv.cartToPolar](../../d2/de8/group__core__array.html#gac5f92f48ec32cacf5275969c33ee837d "../../d2/de8/group__core__array.html#gac5f92f48ec32cacf5275969c33ee837d")(gx, gy)
 bins = np.int32(bin\_n\*ang/(2\*np.pi)) # quantizing binvalues in (0...16)
 bin\_cells = bins[:10,:10], bins[10:,:10], bins[:10,10:], bins[10:,10:]
 mag\_cells = mag[:10,:10], mag[10:,:10], mag[:10,10:], mag[10:,10:]
 hists = [np.bincount(b.ravel(), m.ravel(), bin\_n) for b, m in zip(bin\_cells, mag\_cells)]
 hist = np.hstack(hists) # hist is a 64 bit vector
 return hist
[cv::cartToPolar](../../d2/de8/group__core__array.html#gac5f92f48ec32cacf5275969c33ee837d "../../d2/de8/group__core__array.html#gac5f92f48ec32cacf5275969c33ee837d")void cartToPolar(InputArray x, InputArray y, OutputArray magnitude, OutputArray angle, bool angleInDegrees=false)Calculates the magnitude and angle of 2D vectors.
[cv::Sobel](../../d4/d86/group__imgproc__filter.html#gacea54f142e81b6758cb6f375ce782c8d "../../d4/d86/group__imgproc__filter.html#gacea54f142e81b6758cb6f375ce782c8d")void Sobel(InputArray src, OutputArray dst, int ddepth, int dx, int dy, int ksize=3, double scale=1, double delta=0, int borderType=BORDER\_DEFAULT)Calculates the first, second, third, or mixed image derivatives using an extended Sobel operator.
Finally, as in the previous case, we start by splitting our big dataset into individual cells. For every digit, 250 cells are reserved for training data and remaining 250 data is reserved for testing. Full code is given below, you also can download it from [here](https://github.com/opencv/opencv/tree/4.x/samples/python/tutorial_code/ml/py_svm_opencv/hogsvm.py "https://github.com/opencv/opencv/tree/4.x/samples/python/tutorial_code/ml/py_svm_opencv/hogsvm.py"):

#!/usr/bin/env python

import cv2 as cv
import numpy as np

SZ=20
bin\_n = 16 # Number of bins

affine\_flags = cv.WARP\_INVERSE\_MAP|cv.INTER\_LINEAR

def deskew(img):
 m = [cv.moments](../../d3/dc0/group__imgproc__shape.html#ga556a180f43cab22649c23ada36a8a139 "../../d3/dc0/group__imgproc__shape.html#ga556a180f43cab22649c23ada36a8a139")(img)
 if abs(m['mu02']) < 1e-2:
 return img.copy()
 skew = m['mu11']/m['mu02']
 M = np.float32([[1, skew, -0.5\*SZ\*skew], [0, 1, 0]])
 img = [cv.warpAffine](../../da/d54/group__imgproc__transform.html#ga0203d9ee5fcd28d40dbc4a1ea4451983 "../../da/d54/group__imgproc__transform.html#ga0203d9ee5fcd28d40dbc4a1ea4451983")(img,M,(SZ, SZ),flags=affine\_flags)
 return img

def hog(img):
 gx = [cv.Sobel](../../d4/d86/group__imgproc__filter.html#gacea54f142e81b6758cb6f375ce782c8d "../../d4/d86/group__imgproc__filter.html#gacea54f142e81b6758cb6f375ce782c8d")(img, cv.CV\_32F, 1, 0)
 gy = [cv.Sobel](../../d4/d86/group__imgproc__filter.html#gacea54f142e81b6758cb6f375ce782c8d "../../d4/d86/group__imgproc__filter.html#gacea54f142e81b6758cb6f375ce782c8d")(img, cv.CV\_32F, 0, 1)
 mag, ang = [cv.cartToPolar](../../d2/de8/group__core__array.html#gac5f92f48ec32cacf5275969c33ee837d "../../d2/de8/group__core__array.html#gac5f92f48ec32cacf5275969c33ee837d")(gx, gy)
 bins = np.int32(bin\_n\*ang/(2\*np.pi)) # quantizing binvalues in (0...16)
 bin\_cells = bins[:10,:10], bins[10:,:10], bins[:10,10:], bins[10:,10:]
 mag\_cells = mag[:10,:10], mag[10:,:10], mag[:10,10:], mag[10:,10:]
 hists = [np.bincount(b.ravel(), m.ravel(), bin\_n) for b, m in zip(bin\_cells, mag\_cells)]
 hist = np.hstack(hists) # hist is a 64 bit vector
 return hist

img = [cv.imread](../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8 "../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8")([cv.samples.findFile](../../d6/dba/group__core__utils__samples.html#ga3a33b00033b46c698ff6340d95569c13 "../../d6/dba/group__core__utils__samples.html#ga3a33b00033b46c698ff6340d95569c13")('digits.png'),0)
if img is None:
 raise Exception("we need the digits.png image from samples/data here !")

cells = [np.hsplit(row,100) for row in np.vsplit(img,50)]

# First half is trainData, remaining is testData
train\_cells = [ i[:50] for i in cells ]
test\_cells = [ i[50:] for i in cells]

deskewed = [list(map(deskew,row)) for row in train\_cells]
hogdata = [list(map(hog,row)) for row in deskewed]
trainData = np.float32(hogdata).reshape(-1,64)
responses = np.repeat(np.arange(10),250)[:,np.newaxis]

svm = cv.ml.SVM\_create()
svm.setKernel(cv.ml.SVM\_LINEAR)
svm.setType(cv.ml.SVM\_C\_SVC)
svm.setC(2.67)
svm.setGamma(5.383)

svm.train(trainData, cv.ml.ROW\_SAMPLE, responses)
svm.save('svm\_data.dat')

deskewed = [list(map(deskew,row)) for row in test\_cells]
hogdata = [list(map(hog,row)) for row in deskewed]
testData = np.float32(hogdata).reshape(-1,bin\_n\*4)
result = svm.predict(testData)[1]

mask = result==responses
correct = np.count\_nonzero(mask)
print(correct\*100.0/result.size)
[cv::samples::findFile](../../d6/dba/group__core__utils__samples.html#ga3a33b00033b46c698ff6340d95569c13 "../../d6/dba/group__core__utils__samples.html#ga3a33b00033b46c698ff6340d95569c13")cv::String findFile(const cv::String &relative\_path, bool required=true, bool silentMode=false)Try to find requested data file.
[cv::imread](../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8 "../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8")CV\_EXPORTS\_W Mat imread(const String &filename, int flags=IMREAD\_COLOR)Loads an image from a file.
This particular technique gave me nearly 94% accuracy. You can try different values for various parameters of SVM to check if higher accuracy is possible. Or you can read technical papers on this area and try to implement them.

# Additional Resources

1. [Histograms of Oriented Gradients Video](https://www.youtube.com/watch?v=0Zib1YEE4LU "https://www.youtube.com/watch?v=0Zib1YEE4LU")

# Exercises

1. OpenCV samples contain digits.py which applies a slight improvement of the above method to get improved result. It also contains the reference. Check it and understand it.

