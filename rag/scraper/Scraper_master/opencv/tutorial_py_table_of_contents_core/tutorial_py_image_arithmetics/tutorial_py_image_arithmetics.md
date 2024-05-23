
# Goal

* Learn several arithmetic operations on images, like addition, subtraction, bitwise operations, and etc.
* Learn these functions: **[cv.add()](../../d2/de8/group__core__array.html#ga10ac1bfb180e2cfda1701d06c24fdbd6 "Calculates the per-element sum of two arrays or an array and a scalar.")**, **[cv.addWeighted()](../../d2/de8/group__core__array.html#gafafb2513349db3bcff51f54ee5592a19 "Calculates the weighted sum of two arrays.")**, etc.

# Image Addition

You can add two images with the OpenCV function, [cv.add()](../../d2/de8/group__core__array.html#ga10ac1bfb180e2cfda1701d06c24fdbd6 "Calculates the per-element sum of two arrays or an array and a scalar."), or simply by the numpy operation res = img1 + img2. Both images should be of same depth and type, or the second image can just be a scalar value.

NoteThere is a difference between OpenCV addition and Numpy addition. OpenCV addition is a saturated operation while Numpy addition is a modulo operation.
For example, consider the below sample: 

>>> x = np.uint8([250])
>>> y = np.uint8([10])

>>> print( [cv.add](../../d2/de8/group__core__array.html#ga10ac1bfb180e2cfda1701d06c24fdbd6 "../../d2/de8/group__core__array.html#ga10ac1bfb180e2cfda1701d06c24fdbd6")(x,y) ) # 250+10 = 260 => 255
[[255]]

>>> print( x+y ) # 250+10 = 260 % 256 = 4
[4]
[cv::add](../../d2/de8/group__core__array.html#ga10ac1bfb180e2cfda1701d06c24fdbd6 "../../d2/de8/group__core__array.html#ga10ac1bfb180e2cfda1701d06c24fdbd6")void add(InputArray src1, InputArray src2, OutputArray dst, InputArray mask=noArray(), int dtype=-1)Calculates the per-element sum of two arrays or an array and a scalar.
 This will be more visible when you add two images. Stick with OpenCV functions, because they will provide a better result.

# Image Blending

This is also image addition, but different weights are given to images in order to give a feeling of blending or transparency. Images are added as per the equation below:

\[g(x) = (1 - \alpha)f\_{0}(x) + \alpha f\_{1}(x)\]

By varying \(\alpha\) from \(0 \rightarrow 1\), you can perform a cool transition between one image to another.

Here I took two images to blend together. The first image is given a weight of 0.7 and the second image is given 0.3. [cv.addWeighted()](../../d2/de8/group__core__array.html#gafafb2513349db3bcff51f54ee5592a19 "Calculates the weighted sum of two arrays.") applies the following equation to the image:

\[dst = \alpha \cdot img1 + \beta \cdot img2 + \gamma\]

Here \(\gamma\) is taken as zero. 

img1 = [cv.imread](../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8 "../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8")('ml.png')
img2 = [cv.imread](../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8 "../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8")('opencv-logo.png')
assert img1 is not None, "file could not be read, check with os.path.exists()"
assert img2 is not None, "file could not be read, check with os.path.exists()"

dst = [cv.addWeighted](../../d2/de8/group__core__array.html#gafafb2513349db3bcff51f54ee5592a19 "../../d2/de8/group__core__array.html#gafafb2513349db3bcff51f54ee5592a19")(img1,0.7,img2,0.3,0)

[cv.imshow](../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563 "../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563")('dst',dst)
[cv.waitKey](../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7 "../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7")(0)
[cv.destroyAllWindows](../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481 "../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481")()
[cv::addWeighted](../../d2/de8/group__core__array.html#gafafb2513349db3bcff51f54ee5592a19 "../../d2/de8/group__core__array.html#gafafb2513349db3bcff51f54ee5592a19")void addWeighted(InputArray src1, double alpha, InputArray src2, double beta, double gamma, OutputArray dst, int dtype=-1)Calculates the weighted sum of two arrays.
[cv::imshow](../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563 "../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563")void imshow(const String &winname, InputArray mat)Displays an image in the specified window.
[cv::waitKey](../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7 "../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7")int waitKey(int delay=0)Waits for a pressed key.
[cv::destroyAllWindows](../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481 "../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481")void destroyAllWindows()Destroys all of the HighGUI windows.
[cv::imread](../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8 "../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8")CV\_EXPORTS\_W Mat imread(const String &filename, int flags=IMREAD\_COLOR)Loads an image from a file.
 Check the result below:

![](../../blending.jpg)

image
# Bitwise Operations

This includes the bitwise AND, OR, NOT, and XOR operations. They will be highly useful while extracting any part of the image (as we will see in coming chapters), defining and working with non-rectangular ROI's, and etc. Below we will see an example of how to change a particular region of an image.

I want to put the OpenCV logo above an image. If I add two images, it will change the color. If I blend them, I get a transparent effect. But I want it to be opaque. If it was a rectangular region, I could use ROI as we did in the last chapter. But the OpenCV logo is a not a rectangular shape. So you can do it with bitwise operations as shown below: 

# Load two images
img1 = [cv.imread](../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8 "../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8")('messi5.jpg')
img2 = [cv.imread](../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8 "../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8")('opencv-logo-white.png')
assert img1 is not None, "file could not be read, check with os.path.exists()"
assert img2 is not None, "file could not be read, check with os.path.exists()"

# I want to put logo on top-left corner, So I create a ROI
rows,cols,channels = img2.shape
roi = img1[0:rows, 0:cols]

# Now create a mask of logo and create its inverse mask also
img2gray = [cv.cvtColor](../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab "../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab")(img2,cv.COLOR\_BGR2GRAY)
ret, mask = [cv.threshold](../../d7/d1b/group__imgproc__misc.html#gae8a4a146d1ca78c626a53577199e9c57 "../../d7/d1b/group__imgproc__misc.html#gae8a4a146d1ca78c626a53577199e9c57")(img2gray, 10, 255, cv.THRESH\_BINARY)
mask\_inv = [cv.bitwise\_not](../../d2/de8/group__core__array.html#ga0002cf8b418479f4cb49a75442baee2f "../../d2/de8/group__core__array.html#ga0002cf8b418479f4cb49a75442baee2f")(mask)

# Now black-out the area of logo in ROI
img1\_bg = [cv.bitwise\_and](../../d2/de8/group__core__array.html#ga60b4d04b251ba5eb1392c34425497e14 "../../d2/de8/group__core__array.html#ga60b4d04b251ba5eb1392c34425497e14")(roi,roi,mask = mask\_inv)

# Take only region of logo from logo image.
img2\_fg = [cv.bitwise\_and](../../d2/de8/group__core__array.html#ga60b4d04b251ba5eb1392c34425497e14 "../../d2/de8/group__core__array.html#ga60b4d04b251ba5eb1392c34425497e14")(img2,img2,mask = mask)

# Put logo in ROI and modify the main image
dst = [cv.add](../../d2/de8/group__core__array.html#ga10ac1bfb180e2cfda1701d06c24fdbd6 "../../d2/de8/group__core__array.html#ga10ac1bfb180e2cfda1701d06c24fdbd6")(img1\_bg,img2\_fg)
img1[0:rows, 0:cols ] = dst

[cv.imshow](../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563 "../../d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563")('res',img1)
[cv.waitKey](../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7 "../../d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7")(0)
[cv.destroyAllWindows](../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481 "../../d7/dfc/group__highgui.html#ga6b7fc1c1a8960438156912027b38f481")()
[cv::bitwise\_not](../../d2/de8/group__core__array.html#ga0002cf8b418479f4cb49a75442baee2f "../../d2/de8/group__core__array.html#ga0002cf8b418479f4cb49a75442baee2f")void bitwise\_not(InputArray src, OutputArray dst, InputArray mask=noArray())Inverts every bit of an array.
[cv::bitwise\_and](../../d2/de8/group__core__array.html#ga60b4d04b251ba5eb1392c34425497e14 "../../d2/de8/group__core__array.html#ga60b4d04b251ba5eb1392c34425497e14")void bitwise\_and(InputArray src1, InputArray src2, OutputArray dst, InputArray mask=noArray())computes bitwise conjunction of the two arrays (dst = src1 & src2) Calculates the per-element bit-wis...
[cv::cvtColor](../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab "../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab")void cvtColor(InputArray src, OutputArray dst, int code, int dstCn=0)Converts an image from one color space to another.
[cv::threshold](../../d7/d1b/group__imgproc__misc.html#gae8a4a146d1ca78c626a53577199e9c57 "../../d7/d1b/group__imgproc__misc.html#gae8a4a146d1ca78c626a53577199e9c57")double threshold(InputArray src, OutputArray dst, double thresh, double maxval, int type)Applies a fixed-level threshold to each array element.
 See the result below. Left image shows the mask we created. Right image shows the final result. For more understanding, display all the intermediate images in the above code, especially img1\_bg and img2\_fg.

![](../../overlay.jpg)

image
# Additional Resources

# Exercises

1. Create a slide show of images in a folder with smooth transition between images using [cv.addWeighted](../../d2/de8/group__core__array.html#gafafb2513349db3bcff51f54ee5592a19 "Calculates the weighted sum of two arrays.") function

