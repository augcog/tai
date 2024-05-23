
# Goals

In this chapter, you will learn

* To find objects in an image using Template Matching
* You will see these functions : **[cv.matchTemplate()](../../df/dfb/group__imgproc__object.html#ga586ebfb0a7fb604b35a23d85391329be "Compares a template against overlapped image regions.")**, **[cv.minMaxLoc()](../../d2/de8/group__core__array.html#gab473bf2eb6d14ff97e89b355dac20707 "Finds the global minimum and maximum in an array.")**

# Theory

Template Matching is a method for searching and finding the location of a template image in a larger image. OpenCV comes with a function **[cv.matchTemplate()](../../df/dfb/group__imgproc__object.html#ga586ebfb0a7fb604b35a23d85391329be "Compares a template against overlapped image regions.")** for this purpose. It simply slides the template image over the input image (as in 2D convolution) and compares the template and patch of input image under the template image. Several comparison methods are implemented in OpenCV. (You can check docs for more details). It returns a grayscale image, where each pixel denotes how much does the neighbourhood of that pixel match with template.

If input image is of size (WxH) and template image is of size (wxh), output image will have a size of (W-w+1, H-h+1). Once you got the result, you can use **[cv.minMaxLoc()](../../d2/de8/group__core__array.html#gab473bf2eb6d14ff97e89b355dac20707 "Finds the global minimum and maximum in an array.")** function to find where is the maximum/minimum value. Take it as the top-left corner of rectangle and take (w,h) as width and height of the rectangle. That rectangle is your region of template.

NoteIf you are using [cv.TM\_SQDIFF](../../df/dfb/group__imgproc__object.html#gga3a7850640f1fe1f58fe91a2d7583695dab65c042ed62c9e9e095a1e7e41fe2773 "../../df/dfb/group__imgproc__object.html#gga3a7850640f1fe1f58fe91a2d7583695dab65c042ed62c9e9e095a1e7e41fe2773") as comparison method, minimum value gives the best match.
# Template Matching in OpenCV

Here, as an example, we will search for Messi's face in his photo. So I created a template as below:

![](../../messi_face.jpg)

image
We will try all the comparison methods so that we can see how their results look like: 

import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt

img = [cv.imread](../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8 "../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8")('messi5.jpg', cv.IMREAD\_GRAYSCALE)
assert img is not None, "file could not be read, check with os.path.exists()"
img2 = img.copy()
template = [cv.imread](../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8 "../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8")('template.jpg', cv.IMREAD\_GRAYSCALE)
assert template is not None, "file could not be read, check with os.path.exists()"
w, h = template.shape[::-1]

# All the 6 methods for comparison in a list
methods = ['cv.TM\_CCOEFF', 'cv.TM\_CCOEFF\_NORMED', 'cv.TM\_CCORR',
 'cv.TM\_CCORR\_NORMED', 'cv.TM\_SQDIFF', 'cv.TM\_SQDIFF\_NORMED']

for meth in methods:
 img = img2.copy()
 method = eval(meth)

 # Apply template Matching
 res = [cv.matchTemplate](../../df/dfb/group__imgproc__object.html#ga586ebfb0a7fb604b35a23d85391329be "../../df/dfb/group__imgproc__object.html#ga586ebfb0a7fb604b35a23d85391329be")(img,template,method)
 min\_val, max\_val, min\_loc, max\_loc = [cv.minMaxLoc](../../d2/de8/group__core__array.html#gab473bf2eb6d14ff97e89b355dac20707 "../../d2/de8/group__core__array.html#gab473bf2eb6d14ff97e89b355dac20707")(res)

 # If the method is TM\_SQDIFF or TM\_SQDIFF\_NORMED, take minimum
 if method in [cv.TM\_SQDIFF, cv.TM\_SQDIFF\_NORMED]:
 top\_left = min\_loc
 else:
 top\_left = max\_loc
 bottom\_right = (top\_left[0] + w, top\_left[1] + h)

 [cv.rectangle](../../d6/d6e/group__imgproc__draw.html#ga07d2f74cadcf8e305e810ce8eed13bc9 "../../d6/d6e/group__imgproc__draw.html#ga07d2f74cadcf8e305e810ce8eed13bc9")(img,top\_left, bottom\_right, 255, 2)

 plt.subplot(121),plt.imshow(res,cmap = 'gray')
 plt.title('Matching Result'), plt.xticks([]), plt.yticks([])
 plt.subplot(122),plt.imshow(img,cmap = 'gray')
 plt.title('Detected Point'), plt.xticks([]), plt.yticks([])
 plt.suptitle(meth)

 plt.show()
[cv::minMaxLoc](../../d2/de8/group__core__array.html#gab473bf2eb6d14ff97e89b355dac20707 "../../d2/de8/group__core__array.html#gab473bf2eb6d14ff97e89b355dac20707")void minMaxLoc(InputArray src, double \*minVal, double \*maxVal=0, Point \*minLoc=0, Point \*maxLoc=0, InputArray mask=noArray())Finds the global minimum and maximum in an array.
[cv::imread](../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8 "../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8")CV\_EXPORTS\_W Mat imread(const String &filename, int flags=IMREAD\_COLOR)Loads an image from a file.
[cv::rectangle](../../d6/d6e/group__imgproc__draw.html#ga07d2f74cadcf8e305e810ce8eed13bc9 "../../d6/d6e/group__imgproc__draw.html#ga07d2f74cadcf8e305e810ce8eed13bc9")void rectangle(InputOutputArray img, Point pt1, Point pt2, const Scalar &color, int thickness=1, int lineType=LINE\_8, int shift=0)Draws a simple, thick, or filled up-right rectangle.
[cv::matchTemplate](../../df/dfb/group__imgproc__object.html#ga586ebfb0a7fb604b35a23d85391329be "../../df/dfb/group__imgproc__object.html#ga586ebfb0a7fb604b35a23d85391329be")void matchTemplate(InputArray image, InputArray templ, OutputArray result, int method, InputArray mask=noArray())Compares a template against overlapped image regions.
 See the results below:

* [cv.TM\_CCOEFF](../../df/dfb/group__imgproc__object.html#gga3a7850640f1fe1f58fe91a2d7583695dac5babb7dfda59544e3e31ea928f8cb16 "../../df/dfb/group__imgproc__object.html#gga3a7850640f1fe1f58fe91a2d7583695dac5babb7dfda59544e3e31ea928f8cb16")

![](../../template_ccoeff_1.jpg)

image
* [cv.TM\_CCOEFF\_NORMED](../../df/dfb/group__imgproc__object.html#gga3a7850640f1fe1f58fe91a2d7583695dac6677e2af5e0fae82cc5339bfaef5038 "../../df/dfb/group__imgproc__object.html#gga3a7850640f1fe1f58fe91a2d7583695dac6677e2af5e0fae82cc5339bfaef5038")

![](../../template_ccoeffn_2.jpg)

image
* [cv.TM\_CCORR](../../df/dfb/group__imgproc__object.html#gga3a7850640f1fe1f58fe91a2d7583695da5be00b45a4d99b5e42625b4400bfde65 "../../df/dfb/group__imgproc__object.html#gga3a7850640f1fe1f58fe91a2d7583695da5be00b45a4d99b5e42625b4400bfde65")

![](../../template_ccorr_3.jpg)

image
* [cv.TM\_CCORR\_NORMED](../../df/dfb/group__imgproc__object.html#gga3a7850640f1fe1f58fe91a2d7583695daf9c3ab9296f597ea71f056399a5831da "../../df/dfb/group__imgproc__object.html#gga3a7850640f1fe1f58fe91a2d7583695daf9c3ab9296f597ea71f056399a5831da")

![](../../template_ccorrn_4.jpg)

image
* [cv.TM\_SQDIFF](../../df/dfb/group__imgproc__object.html#gga3a7850640f1fe1f58fe91a2d7583695dab65c042ed62c9e9e095a1e7e41fe2773 "../../df/dfb/group__imgproc__object.html#gga3a7850640f1fe1f58fe91a2d7583695dab65c042ed62c9e9e095a1e7e41fe2773")

![](../../template_sqdiff_5.jpg)

image
* [cv.TM\_SQDIFF\_NORMED](../../df/dfb/group__imgproc__object.html#gga3a7850640f1fe1f58fe91a2d7583695da5382c8f9df87e87cf1e9f9927dc3bc31 "../../df/dfb/group__imgproc__object.html#gga3a7850640f1fe1f58fe91a2d7583695da5382c8f9df87e87cf1e9f9927dc3bc31")

![](../../template_sqdiffn_6.jpg)

image
You can see that the result using **[cv.TM\_CCORR](../../df/dfb/group__imgproc__object.html#gga3a7850640f1fe1f58fe91a2d7583695da5be00b45a4d99b5e42625b4400bfde65 "../../df/dfb/group__imgproc__object.html#gga3a7850640f1fe1f58fe91a2d7583695da5be00b45a4d99b5e42625b4400bfde65")** is not good as we expected.

# Template Matching with Multiple Objects

In the previous section, we searched image for Messi's face, which occurs only once in the image. Suppose you are searching for an object which has multiple occurrences, **[cv.minMaxLoc()](../../d2/de8/group__core__array.html#gab473bf2eb6d14ff97e89b355dac20707 "Finds the global minimum and maximum in an array.")** won't give you all the locations. In that case, we will use thresholding. So in this example, we will use a screenshot of the famous game **Mario** and we will find the coins in it. 

import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt

img\_rgb = [cv.imread](../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8 "../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8")('mario.png')
assert img\_rgb is not None, "file could not be read, check with os.path.exists()"
img\_gray = [cv.cvtColor](../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab "../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab")(img\_rgb, cv.COLOR\_BGR2GRAY)
template = [cv.imread](../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8 "../../d4/da8/group__imgcodecs.html#gab32ee19e22660912565f8140d0f675a8")('mario\_coin.png', cv.IMREAD\_GRAYSCALE)
assert template is not None, "file could not be read, check with os.path.exists()"
w, h = template.shape[::-1]

res = [cv.matchTemplate](../../df/dfb/group__imgproc__object.html#ga586ebfb0a7fb604b35a23d85391329be "../../df/dfb/group__imgproc__object.html#ga586ebfb0a7fb604b35a23d85391329be")(img\_gray,template,cv.TM\_CCOEFF\_NORMED)
threshold = 0.8
loc = np.where( res >= threshold)
for pt in zip(\*loc[::-1]):
 [cv.rectangle](../../d6/d6e/group__imgproc__draw.html#ga07d2f74cadcf8e305e810ce8eed13bc9 "../../d6/d6e/group__imgproc__draw.html#ga07d2f74cadcf8e305e810ce8eed13bc9")(img\_rgb, pt, (pt[0] + w, pt[1] + h), (0,0,255), 2)

[cv.imwrite](../../d4/da8/group__imgcodecs.html#ga8ac397bd09e48851665edbe12aa28f25 "../../d4/da8/group__imgcodecs.html#ga8ac397bd09e48851665edbe12aa28f25")('res.png',img\_rgb)
[cv::imwrite](../../d4/da8/group__imgcodecs.html#ga8ac397bd09e48851665edbe12aa28f25 "../../d4/da8/group__imgcodecs.html#ga8ac397bd09e48851665edbe12aa28f25")CV\_EXPORTS\_W bool imwrite(const String &filename, InputArray img, const std::vector< int > &params=std::vector< int >())Saves an image to a specified file.
[cv::cvtColor](../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab "../../d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab")void cvtColor(InputArray src, OutputArray dst, int code, int dstCn=0)Converts an image from one color space to another.
 Result:

![](../../res_mario.jpg)

image
# Additional Resources

# Exercises

