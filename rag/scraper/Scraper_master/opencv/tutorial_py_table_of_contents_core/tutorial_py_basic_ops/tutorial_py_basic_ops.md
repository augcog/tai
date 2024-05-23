
# Goal

Learn to:

* Access pixel values and modify them
* Access image properties
* Set a Region of Interest (ROI)
* Split and merge images

Almost all the operations in this section are mainly related to Numpy rather than OpenCV. A good knowledge of Numpy is required to write better optimized code with OpenCV.

\*( Examples will be shown in a Python terminal, since most of them are just single lines of code )\*

# Accessing and Modifying pixel values

Let's load a color image first: 

>>> import numpy as np
>>> import cv2 as cv

>>> img = [cv.imread](../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8 "../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8")('messi5.jpg')
>>> assert img is not None, "file could not be read, check with os.path.exists()"
[cv::imread](../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8 "../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8")CV\_EXPORTS\_W Mat imread(const String &filename, int flags=IMREAD\_COLOR)Loads an image from a file.
 You can access a pixel value by its row and column coordinates. For BGR image, it returns an array of Blue, Green, Red values. For grayscale image, just corresponding intensity is returned. 

>>> px = img[100,100]
>>> print( px )
[157 166 200]

# accessing only blue pixel
>>> blue = img[100,100,0]
>>> print( blue )
157
 You can modify the pixel values the same way. 

>>> img[100,100] = [255,255,255]
>>> print( img[100,100] )
[255 255 255]
**Warning**

Numpy is an optimized library for fast array calculations. So simply accessing each and every pixel value and modifying it will be very slow and it is discouraged.

NoteThe above method is normally used for selecting a region of an array, say the first 5 rows and last 3 columns. For individual pixel access, the Numpy array methods, array.item() and array.itemset() are considered better. They always return a scalar, however, so if you want to access all the B,G,R values, you will need to call array.item() separately for each value.
Better pixel accessing and editing method : 

# accessing RED value
>>> img.item(10,10,2)
59

# modifying RED value
>>> img.itemset((10,10,2),100)
>>> img.item(10,10,2)
100
# Accessing Image Properties

Image properties include number of rows, columns, and channels; type of image data; number of pixels; etc.

The shape of an image is accessed by img.shape. It returns a tuple of the number of rows, columns, and channels (if the image is color): 

>>> print( img.shape )
(342, 548, 3)
NoteIf an image is grayscale, the tuple returned contains only the number of rows and columns, so it is a good method to check whether the loaded image is grayscale or color.
Total number of pixels is accessed by `img.size`: 

>>> print( img.size )
562248
 Image datatype is obtained by `img.dtype`: 

>>> print( img.dtype )
uint8
Noteimg.dtype is very important while debugging because a large number of errors in OpenCV-Python code are caused by invalid datatype.
# Image ROI

Sometimes, you will have to play with certain regions of images. For eye detection in images, first face detection is done over the entire image. When a face is obtained, we select the face region alone and search for eyes inside it instead of searching the whole image. It improves accuracy (because eyes are always on faces :D ) and performance (because we search in a small area).

ROI is again obtained using Numpy indexing. Here I am selecting the ball and copying it to another region in the image: 

>>> ball = img[280:340, 330:390]
>>> img[273:333, 100:160] = ball
 Check the results below:

![](../../roi.jpg)

image
# Splitting and Merging Image Channels

Sometimes you will need to work separately on the B,G,R channels of an image. In this case, you need to split the BGR image into single channels. In other cases, you may need to join these individual channels to create a BGR image. You can do this simply by: 

>>> b,g,r = [cv.split](../../d2/de8/group__core__array.html#ga0547c7fed86152d7e9d0096029c8518a "../../d2/de8/group__core__array.html#ga0547c7fed86152d7e9d0096029c8518a")(img)
>>> img = [cv.merge](../../d2/de8/group__core__array.html#ga7d7b4d6c6ee504b30a20b1680029c7b4 "../../d2/de8/group__core__array.html#ga7d7b4d6c6ee504b30a20b1680029c7b4")((b,g,r))
[cv::split](../../d2/de8/group__core__array.html#ga0547c7fed86152d7e9d0096029c8518a "../../d2/de8/group__core__array.html#ga0547c7fed86152d7e9d0096029c8518a")void split(const Mat &src, Mat \*mvbegin)Divides a multi-channel array into several single-channel arrays.
[cv::merge](../../d2/de8/group__core__array.html#ga7d7b4d6c6ee504b30a20b1680029c7b4 "../../d2/de8/group__core__array.html#ga7d7b4d6c6ee504b30a20b1680029c7b4")void merge(const Mat \*mv, size\_t count, OutputArray dst)Creates one multi-channel array out of several single-channel ones.
 Or 

>>> b = img[:,:,0]
 Suppose you want to set all the red pixels to zero - you do not need to split the channels first. Numpy indexing is faster: 

>>> img[:,:,2] = 0
**Warning**

[cv.split()](../../d2/de8/group__core__array.html#ga0547c7fed86152d7e9d0096029c8518a "Divides a multi-channel array into several single-channel arrays.") is a costly operation (in terms of time). So use it only if necessary. Otherwise go for Numpy indexing.

# Making Borders for Images (Padding)

If you want to create a border around an image, something like a photo frame, you can use **[cv.copyMakeBorder()](../../d2/de8/group__core__array.html#ga2ac1049c2c3dd25c2b41bffe17658a36 "Forms a border around an image.")**. But it has more applications for convolution operation, zero padding etc. This function takes following arguments:

* **src** - input image
* **top**, **bottom**, **left**, **right** - border width in number of pixels in corresponding directions
* **borderType** - Flag defining what kind of border to be added. It can be following types:
	+ **[cv.BORDER\_CONSTANT](../../d2/de8/group__core__array.html#gga209f2f4869e304c82d07739337eae7c5aed2e4346047e265c8c5a6d0276dcd838 "iiiiii|abcdefgh|iiiiiii with some specified i")** - Adds a constant colored border. The value should be given as next argument.
	+ **[cv.BORDER\_REFLECT](../../d2/de8/group__core__array.html#gga209f2f4869e304c82d07739337eae7c5a815c8a89b7cb206dcba14d11b7560f4b "fedcba|abcdefgh|hgfedcb")** - Border will be mirror reflection of the border elements, like this : *fedcba|abcdefgh|hgfedcb*
	+ **[cv.BORDER\_REFLECT\_101](../../d2/de8/group__core__array.html#gga209f2f4869e304c82d07739337eae7c5ab3c5a6143d8120b95005fa7105a10bb4 "gfedcb|abcdefgh|gfedcba")** or **[cv.BORDER\_DEFAULT](../../d2/de8/group__core__array.html#gga209f2f4869e304c82d07739337eae7c5afe14c13a4ea8b8e3b3ef399013dbae01 "same as BORDER_REFLECT_101")** - Same as above, but with a slight change, like this : *gfedcb|abcdefgh|gfedcba*
	+ **[cv.BORDER\_REPLICATE](../../d2/de8/group__core__array.html#gga209f2f4869e304c82d07739337eae7c5aa1de4cff95e3377d6d0cbe7569bd4e9f "aaaaaa|abcdefgh|hhhhhhh")** - Last element is replicated throughout, like this: *aaaaaa|abcdefgh|hhhhhhh*
	+ **[cv.BORDER\_WRAP](../../d2/de8/group__core__array.html#gga209f2f4869e304c82d07739337eae7c5a697c1b011884a7c2bdc0e5caf7955661 "cdefgh|abcdefgh|abcdefg")** - Can't explain, it will look like this : *cdefgh|abcdefgh|abcdefg*
* **value** - Color of border if border type is [cv.BORDER\_CONSTANT](../../d2/de8/group__core__array.html#gga209f2f4869e304c82d07739337eae7c5aed2e4346047e265c8c5a6d0276dcd838 "iiiiii|abcdefgh|iiiiiii with some specified i")

Below is a sample code demonstrating all these border types for better understanding: 

import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt

BLUE = [255,0,0]

img1 = [cv.imread](../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8 "../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8")('opencv-logo.png')
assert img1 is not None, "file could not be read, check with os.path.exists()"

replicate = [cv.copyMakeBorder](../../d2/de8/group__core__array.html#ga2ac1049c2c3dd25c2b41bffe17658a36 "../../d2/de8/group__core__array.html#ga2ac1049c2c3dd25c2b41bffe17658a36")(img1,10,10,10,10,cv.BORDER\_REPLICATE)
reflect = [cv.copyMakeBorder](../../d2/de8/group__core__array.html#ga2ac1049c2c3dd25c2b41bffe17658a36 "../../d2/de8/group__core__array.html#ga2ac1049c2c3dd25c2b41bffe17658a36")(img1,10,10,10,10,cv.BORDER\_REFLECT)
reflect101 = [cv.copyMakeBorder](../../d2/de8/group__core__array.html#ga2ac1049c2c3dd25c2b41bffe17658a36 "../../d2/de8/group__core__array.html#ga2ac1049c2c3dd25c2b41bffe17658a36")(img1,10,10,10,10,cv.BORDER\_REFLECT\_101)
wrap = [cv.copyMakeBorder](../../d2/de8/group__core__array.html#ga2ac1049c2c3dd25c2b41bffe17658a36 "../../d2/de8/group__core__array.html#ga2ac1049c2c3dd25c2b41bffe17658a36")(img1,10,10,10,10,cv.BORDER\_WRAP)
constant= [cv.copyMakeBorder](../../d2/de8/group__core__array.html#ga2ac1049c2c3dd25c2b41bffe17658a36 "../../d2/de8/group__core__array.html#ga2ac1049c2c3dd25c2b41bffe17658a36")(img1,10,10,10,10,cv.BORDER\_CONSTANT,value=BLUE)

plt.subplot(231),plt.imshow(img1,'gray'),plt.title('ORIGINAL')
plt.subplot(232),plt.imshow(replicate,'gray'),plt.title('REPLICATE')
plt.subplot(233),plt.imshow(reflect,'gray'),plt.title('REFLECT')
plt.subplot(234),plt.imshow(reflect101,'gray'),plt.title('REFLECT\_101')
plt.subplot(235),plt.imshow(wrap,'gray'),plt.title('WRAP')
plt.subplot(236),plt.imshow(constant,'gray'),plt.title('CONSTANT')

plt.show()
[cv::copyMakeBorder](../../d2/de8/group__core__array.html#ga2ac1049c2c3dd25c2b41bffe17658a36 "../../d2/de8/group__core__array.html#ga2ac1049c2c3dd25c2b41bffe17658a36")void copyMakeBorder(InputArray src, OutputArray dst, int top, int bottom, int left, int right, int borderType, const Scalar &value=Scalar())Forms a border around an image.
 See the result below. (Image is displayed with matplotlib. So RED and BLUE channels will be interchanged):

![](../../border.jpg)

image
# Additional Resources

# Exercises

