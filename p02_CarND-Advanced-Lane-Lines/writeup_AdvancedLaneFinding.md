<!-- #region -->
## Writeup : Project02 Advanced Lane Finding

---

**Rubric Points**

In this project, following rubric points are addressed.
 1. Camera Calibration  
 2. Pipeline for images  
  2.1 Distortion correction  
  2.2 Lane extraction  
  2.3 Perspective transform  
  2.4 Lane identification and fitting  
  2.5 Lane curvature calculation  
  2.6 Draw lane region  
 3. Pipeline for videos 
 4. Discussion


[//]: # (Image References)

[image1]: ./writeup/calibration_corner_detected.jpg "Detected"
[image3]: ./writeup/calibration3_undistort.jpg "Undistorted"
[image4]: ./writeup/calibration5_undistort.jpg "Undistorted"
[image5]: ./writeup/undistort_image.jpg "Fit Visual"
[image6]: ./writeup/straight_lines2_undistorted_abssobel.jpg
[image7]: ./writeup/straight_lines2_undistorted_magsobel.jpg
[image8]: ./writeup/straight_lines2_undistorted_dirthresh.jpg
[image9]: ./writeup/straight_lines2_undistorted_rgbsel.jpg
[image10]: ./writeup/straight_lines2_undistorted_hlssel.jpg
[image11]: ./writeup/straight_lines2_undistorted_combo.jpg
[image12]: ./writeup/straight_lines1_warped.jpg
[image13]: ./writeup/straight_lines2_undistorted_fitted.jpg
[image14]: ./writeup/straight_line_draw.jpg
[image15]: ./writeup/test34_draw.jpg

[video1]: ./output_images/project_video_out.mp4 "Video"

---

## 1. Camera Calibration

#### Briefly state how you computed the camera matrix and distortion coefficients. Provide an example of a distortion corrected calibration image.
---

## Brief description of camera calibration

When a camera looks at 3D objects and transformd them into a 2D image, image distortion occurs because that transformation is not perfect. As it is important to measure curvertures of the roads precisely for safe driving, we need to correct distortion of the camera image.
The camera matrix is a transformative matrix which represents the above relationship between the 3D object points and the 2D image points. Distortion coefficients are parameters used to mathematically calcuate the above transformation. There are two different types of distortion coefficients; radial distortion coefficients k1, k2, k3 and tangential distortion coefficients p1, p2. Radial distortion makes straight lines into curved lines, and the effects get larger as the distance from the image center increases. Tangental distortion is generated because image plane does not completely face parallel to camera lens. This distortion makes some region looks closer than it actually is in reality.

The code for this step is contained in the first three code cells of the IPython notebook located in "./P2.ipynb". 

## Detect corner points of each image

I read all 20 images in camera_cal folder and detected 9x6 chessboard corners. As calibration1.jpg did not include all corners in the image, detection results of it was not included in objpoints or imgpoints lists. Detection results were stored in output_images/1.1_detection_result/ folder.

![alt text][image1]

## Find camera matrix and undistort image

Using detected corner points, I calculated camera matrix(mtx) and distortion coefficients(dist). Calibration results were stored in output_images/1.2_camera_calibration_result/ folder. Though two images in the camera_cal folder had different image sizes; 1281x721 for the two images and 1280x720 for the other images, I could not see the effects of the difference. Calibration7/15_undistort2.jpg were the results obtained from 1281x721 parameters.  
To clearly show the calibration result, I choose to include calibration3.jpg and calibration5.jpg in this writeup.  

![alt text][image3]
![alt text][image4]


---

## 2. Pipeline for images

---
## 2.1 Distortion correction 

#### Provide an example of a distortion-corrected image.
---
As all images in test_images folder had the same size 1280x720. I made undistorted images of them using camera matrix and distortion coefficient calculated above. Resulting images were stored in output_images/2.1 distortion_corrected_image/.
The code for this step is contained in the forth code cell of the IPython notebook located in "./P2.ipynb". 

To demonstrate this step, I will describe how I apply the distortion correction to one of the test images like this one:
![alt text][image5]


---
## 2.2 Lane extraction  

#### Describe how (and identify where in your code) you used color transforms, gradients or other methods to create a thresholded binary image. Provide an example of a binary image result.
---
I learned different kinds of filtering methods to select lane pixels. They are 1. absolute sobel threshold, 2. magnitude of sobel threshold, 3. sobel direction threshold, 4. rgb color threshold and 5. hls color threshold. I tried to apply them to the sample images.  
The code for this step is contained in the 5th to 10th code cell of the IPython notebook located in "./P2.ipynb".

#### 2.2.1. Absolute sobel threshold

This method considered derivative of images in x or y direction using sobel filter. It seemed to work for straight_lines images and test images of 2, 3 and 6, but it did not work for test images of 1, 4 and 5. Those images with unsatisfacotry results had whitish road color which made lane line less clear.  
To demonstrate this step, I will show you one of the test images like this one. Here left image is a result of applying sobel of x direction, whereas the right image is that of sobel of y direction.
![alt text][image6]

#### 2.2.2. Magnitude of sobel threshold

This method considered the magnitude of the above sobel derivatives.  
It did to seem to work even for straight_lines images stably.
![alt text][image7]

#### 2.2.3. Sobel direction threshold

This method considered direction of edges using sobel filter.  
As magnitude of sobel threshold, it did not seem to work properly even for simple images.
![alt text][image8]

#### 2.2.4.  RGB color space

This method saw color of Lanes in RGB color space.  
Though this method, especially R threshold, worked well to distinguish lanes for several images, it could not extract lanes for test1, 4, 5 images. Those image had whitish roads region where all RGB had large values. 
![alt text][image9]

#### 2.2.5.  HLS color space

This method considered HLS color, especially Saturation of the images. This filter worked well even for difficult images with whitish roads. This meant whitish region of the roads did not have high saturation.
![alt text][image10]

#### 2.2.6.  Combination

Considering the above examination, 1. absolute sobel and 5. HLS color were effective to extract Lane pixels. Then I combined these two method with 'or' relationship. Resulting images were stored in output_images/2.2_binary.
![alt text][image11]


---
## 2.3 Perspective transform   

#### Describe how (and identify where in your code) you performed a perspective transform and provide an example of a transformed image.
---

As I assumed that the road was flat and the camera perspective had not changed, I selected straight_lines1.jpg for perspective transform. On that image, I picked up four points in a trapezoidal shape lying along the lane line. Then I selected four points in a rectangle shape as a destination, considering that the image size was 1280x720. The result of the perspective transformation was stored in output_images/2.3_perspective/ folder.  
The code for this step is contained in the 11th code cell of the IPython notebook located in "./P2.ipynb".

![alt text][image12]

---
## 2.4 Lane identification and fitting  

#### Describe how (and identify where in your code) you identified lane-line pixels and fit their positions with a polynomial?
---

Finding lane-line pixels and fit them were performed by following codes learned in lesson 8.4. As there were more difficult images included in the sample, lane pixel extraction was unstable. Then, there were cases that no pixels were found in a small window. To avoid those emply windows cause errors, I added error checking codes in line62 and line64 which hindered empty window's empty lists to be added. The results of this lane finding and fitting were stored in output_images/2.4_lane_fit/ folder.  
The code for this step is contained in the 12th code cell of the IPython notebook located in "./P2.ipynb".

![alt text][image13]


---
## 2.5 Lane curvature calculation   

#### Describe how (and identify where in your code) you calculated the radius of curvature of the lane and the position of the vehicle with respect to center.
---

Using the fitting results of 2.4 and formula of lesson 8.6, I calculated curvature of two lanes in sample images. I calculated the curvature at the bottom of the image, because curvatures closest to the vehicle were the most important. Also I used xm_per_pix(meters per pixel in x dimension) = 3.7/680, because I chose to place lanes 680 pixels apart at perspective transformation.  
The code for this step is contained in the 13th code cell of the IPython notebook located in "./P2.ipynb".

Followings were the calculated curvatures of left or right lanes. Though the general trends from the images were correctly reflected, the variability was quite large unfortunately.Especially, difference between curvatures of left and right lanes were larger than expected.

|  image name    | Left Lane     | Right Lane    | 
|:--------------:|:-------------:|:-------------:| 
| straight_lines1| 1576          | 35038         | 
| straight_lines2| 4383          | 8369          |
|     test1      | 620           | 471           |
|     test2      | 473           | 375           |
|     test3      | 1950          | 607           |
|     test4      | 861           | 293           |
|     test5      | 317           | 197           |
|     test6      | 820           | 509           |

---
## 2.6 Draw lane region   

#### Provide an example image of your result plotted back down onto the road such that the lane area is identified clearly.
---
To draw lane line on the original image(after distortion correction, before perspective transformation), I used inverse transformation of perspective transormation. I defined the inverse transform at 2.3. First I used perspective transformed image(imageA) as in 2.4 and 2.5, and I colored left lane pixels in Red and right lane pixels in Blue. Then, I colored in yellow between the lane lines obtained from the fitting in 2.4. After those lane drawing process, I got inverse transformed image(imageB) of imageA which colored left and right lanes of the original image. I blended the imageB to the original image.
The code for this step is contained in the 14th code cell of the IPython notebook located in "./P2.ipynb".
![alt text][image14]
![alt text][image15]


---

## 3. Pipeline for video

#### Provide a link to your final video output. Your pipeline should perform reasonably well on the entire project video (wobbly lines are ok but no catastrophic failures that would cause the car to drive off the road!)

---
I applyed the pipeline made for each to sample video.
The code for this step is contained in the 14th code cell of the IPython notebook located in "./P2.ipynb".

Here's a [link to my video result](./output_images/project_video_out.mp4)

---

## 4. Discussion

#### Briefly discuss any problems / issues you faced in your implementation of this project. Where will your pipeline likely fail? What could you do to make it more robust?

---
I thought the most difficult part of thie project was lane extraction. I was impressed by the effectiveness of HLS color space. However, most of other methods were problematic and did not work robustly. Though edge extraction related methods were popular and easy to understand, the results of these methods were greatly affected by image changes.I also felt that there was no stemming method to search for the optimal solution, and that trial and error was necessary.

I was also worried that the calculated curvature of the road had large variations.In particular, the calculation results were often different between the left and right lanes. However, I believed that the cause of these instabilities was the instability of the lane detection results described above. This time, only the process of taking the average of the left and right calculation results was performed, but I think that there was room for improvement such as referring to the values of the previous and next frames.

If I were going to pursue this project further, I would like to consider a method that did not depend on lane detection. 
<!-- #endregion -->

```python

```
