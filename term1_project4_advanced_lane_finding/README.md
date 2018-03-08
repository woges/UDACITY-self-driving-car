# Advanced Lane Finding
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

## Overview

A lane detection and tracking program that uses a traditional (i.e. non-machine-learning) computer vision approach to detect lane lines is implemented here.

![T1P4](./results/white_giphy.gif)

## Dependencies

If you have already installed all the necessary dependencies for the projects in term 1 you should be good to go! If not, you should install them to get started on this project => [Getting Started for Term 1](../term1_How_to_get_started). 
 
## Basic Build Instructions

1. Clone or fork this repository.
2. Launch the Jupyter notebook: `jupyter notebook P4_adv_lane_lines.ipynb`
3. Execute the code cells you are interested in. 
Note that cells may depend on previous cells. The notebook explains clearly what each code cell does.

## Goal of this project

The goal of this project is to develop a pipeline to process a video stream from a forward-facing camera mounted on the front of a car, and output an annotated video which identifies:
- The positions of the lane lines 
- The location of the vehicle relative to the center of the lane
- The radius of curvature of the road

## Advanced Lane Detection Pipeline

The pipeline created for this project processes images in the following steps:
- **Step 0**: Compute the camera calibration matrix and distortion coefficients given a set of chessboard images. 
- **Step 1**: Apply distortion correction using a calculated camera calibration matrix and distortion coefficients.
- **Step 2**: Apply color thresholds to create a binary image which isolates the pixels representing lane lines.
- **Step 3**: Apply a perspective transformation to warp the image to a birds eye view perspective of the lane lines.
- **Step 4**: Identify the lane line pixels and fit polynomials to the lane boundaries.
- **Step 5**: Determine curvature of the lane and vehicle position with respect to center.
- **Step 6**: Warp the detected lane boundaries back onto the original image.
- **Step 7**: Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

### Step 0: Camera Calibration (Preparation for pipeline)
Image distortion occurs when a camera looks at 3D objects in real world and transforms them into a 2D image - this transformation isn’t perfect. Distortion actually changes what the shape and size of these 3D objects appear to be. The reason for this is that we use lenses in our camera system. They cause the light rays often to bend a little too much or too little at the edges of these lenses.  Another type of distortion is the tangential distortion, which occurs when the lenses aren't aligned perfectly parallel to the imaging plane. 
So, the first step in analyzing camera images, is to undo this distortion so that you can get correct and useful information out of them. Distortion correction is very important in the field of surveillance of the environment with optical systems as distortion (if not corrected) changes or makes:  

  - apparent size of an object in an image 
  - apparent shape of an object in an image
  - an object's appearance depending on where it's in the field of view
  - object's appear closer/farther away than they actually are.
 
This is best done using an chessboard as this is regular, has a high contrast pattern, is easy to detect automatically and we know what a undistorted chessboard looks like. We should use at least 20 images taken from different angles and distances to get a reliable calibration. At last a test images should be used to check the undistortion.

OpenCV provide some really helpful built-in functions for the task on camera calibration. First of all, to detect the calibration pattern in the [calibration images](./camera_cal/), we can use the function `cv2.findChessboardCorners(image, pattern_size)`. identify the locations of corners on a series of pictures of a chessboard taken from different angles.

![dist_undist_images](./img/dist_undist_images.png)

Once we have stored the correspondeces between 3D world and 2D image points for a bunch of images, we can proceed to actually calibrate the camera through `cv2.calibrateCamera()`. The locations of the chessboard corners were used as input to this OpenCV function. Among other things, this function returns both the *camera matrix* and the *distortion coefficients*, which we can use to undistort the frames.

### Pipeline (single images)
### Step 1: Distortion Correction

Once the camera is calibrated, we can use the camera matrix and distortion coefficients we found to remove distortion from highway driving images. Indeed, if we want to study the *geometry* of the road, we have to be sure that the images we're processing do not present distortions. Here's the result of distortion-correction on one of the test images:

![dist_undist_highway_01](./img/dist_undist_highway_01.png)

![dist_undist_highway_02](./img/dist_undist_highway_02.png)

Notice that if you compare each two images, especially around the edges, there are obvious differences between the original and undistorted image, indicating that distortion has been removed from the original image.

### Step 2: Create a thresholded binary image

Correctly creating the binary image from the input frame is the very first step of the whole pipeline that will lead us to detect the lane. For this reason that is also one of the most important steps. If the binary image is bad, it's very difficult to recover and to obtain good results in the successive steps of the pipeline.

There are several threshold functions which could be used to generate a binary image and detect lane lines also under worse conditions like shadows and lower brightness. Here different function were used to manage this task.

**1. Region of Interest**
  This defines a mask (polygonial shape) in the image where you normally expect the lanes to be. The rest of the image is set to black.

**2. Sobel Operator**
  With canny-edge detection we found pixels in the image that were likely to be part of a line in project 1 'lane finding'. Here we use the sobel operator as we know that the lines we are looking for are close to vertical.  Applying the sobel operator to an images is a way of taking the derivative of the image in the x or y direction. With the results of the sobel operation the gradient in x or y direction is calculated Afterwards a threshold is applied to identify pixels within a certain gradient range.

**3. Magnitude of the gradient**
  The magnitude of the gradient is the square root of the sum of the squares in each direction, which ca´n also be thresholded.

**4. Direction of the gradient**
  Another possibility to identify the lane lines more clearly is to select only lines with a certain orientation in the image. We can calculate the direction of the gradient by simply using the inverse tangent of the y gradient divided by the x gradient.

**5. Color channel**
  Besides those options a simple threshold to the color channel can be applied.

**6. Differnet Color spaces**
  Also using different color channels or different color spaces can be very useful.

**7. Combination of all above**
  A combination of different binary thresholds, to create one combination thresholded image does a great job of highlighting almost all of the white and yellow lane lines.

I used a combination of color and gradient thresholds to generate a binary image. In order to detect the white lines, I found that [equalizing the histogram](http://docs.opencv.org/3.1.0/d5/daf/tutorial_py_histogram_equalization.html) of the input frame before thresholding works really well to highlight the actual lane lines. For the yellow lines, I employed a threshold on V channel in [HSV](http://docs.opencv.org/3.2.0/df/d9d/tutorial_py_colorspaces.html) color space. Furthermore, I also convolve the input frame with Sobel kernel to get an estimate of the gradients of the lines. Finally, I make use of [morphological closure](http://docs.opencv.org/3.0-beta/doc/py_tutorials/py_imgproc/py_morphological_ops/py_morphological_ops.html) to *fill the gaps* in my binary image. Here I show every substep and the final output:

<p align="center">
  <img src="./img/threshold_01.png" width="960">
</p>

<p align="center">
  <img src="./img/threshold_02.png" width="960">
</p>

<p align="center">
  <img src="./img/threshold_03.png" width="960">
</p>

### Step 2: Perspective Transform
The goal of this step is to transform the undistorted image to a "birds eye view" of the road which focuses only on the lane lines and displays them in such a way that they appear to be relatively parallel to eachother (as opposed to the converging lines you would normally see). To achieve the perspective transformation the OpenCV functions `getPerspectiveTransform` and `warpPerspective` were applied which take a matrix of four source points on the undistorted image and remaps them to four destination points on the warped image. The source and destination points were selected manually by visualizing the locations of the lane lines on a series of test images.

The following image shows the original image with the rectangle source points on the left side and the undistorted and warped image with the destination points on the right side. As this is a straight lane the lines appear as parallel vertical lines in the result image.

<p align="left">
  <img src="./img/ori_warped_01.png" width="480">
</p>
<p align="right">
  <img src="./img/ori_warped_02.png" width="480">
</p>

Below an other example of a curved lane is shown, with a binary thresholded and warped image, as it is needed for the further steps:

<p align="center">
  <img src="./img/ori_warped_03.png" width="480">
</p>

### Steps 4, 5 and 6: Fitting a polynomial to the lane lines, calculating vehicle position and radius of curvature:
At this point I was able to use the combined binary image to isolate only the pixels belonging to lane lines. The next step was to fit a polynomial to each lane line, which was done by:
- Identifying peaks in a histogram of the image to determine location of lane lines.
- Identifying all non zero pixels around histogram peaks using the numpy function `numpy.nonzero()`.
- Fitting a polynomial to each lane using the numpy function `numpy.polyfit()`.

After fitting the polynomials I was able to calculate the position of the vehicle with respect to center with the following calculations:
- Calculated the average of the x intercepts from each of the two polynomials `position = (rightx_int+leftx_int)/2`
- Calculated the distance from center by taking the absolute value of the vehicle position minus the halfway point along the horizontal axis `distance_from_center = abs(image_width/2 - position)`
- If the horizontal position of the car was greater than `image_width/2` than the car was considered to be left of center, otherwise right of center.
- Finally, the distance from center was converted from pixels to meters by multiplying the number of pixels by `3.7/700`.

Next I used the following code to calculate the radius of curvature for each lane line in meters:
```
ym_per_pix = 30./720 # meters per pixel in y dimension
xm_per_pix = 3.7/700 # meteres per pixel in x dimension
left_fit_cr = np.polyfit(lefty*ym_per_pix, leftx*xm_per_pix, 2)
right_fit_cr = np.polyfit(righty*ym_per_pix, rightx*xm_per_pix, 2)
left_curverad = ((1 + (2*left_fit_cr[0]*np.max(lefty) + left_fit_cr[1])**2)**1.5) \
                             /np.absolute(2*left_fit_cr[0])
right_curverad = ((1 + (2*right_fit_cr[0]*np.max(lefty) + right_fit_cr[1])**2)**1.5) \
                                /np.absolute(2*right_fit_cr[0])
```
*Source:* http://www.intmath.com/applications-differentiation/8-radius-curvature.php

The final radius of curvature was taken by average the left and right curve radiuses.

### Step 7: Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.
The final step in processing the images was to plot the polynomials on to the warped image, fill the space between the polynomials to highlight the lane that the car is in, use another perspective trasformation to unwarp the image from birds eye back to its original perspective, and print the distance from center and radius of curvature on to the final annotated image.

![Filled Image](./images/filled.png)

## Video Processing Pipeline:
After establishing a pipeline to process still images, the final step was to expand the pipeline to process videos frame-by-frame, to simulate what it would be like to process an image stream in real time on an actual vehicle. 

My goal in developing a video processing pipeline was to create as smooth of an output as possible. To achieve this, I created a class for each of the left and right lane lines and stored features of each lane for averaging across frames.

The video pipeline first checks whether or not the lane was detected in the previous frame. If it was, then it only checks for lane pixels in close proximity to the polynomial calculated in the previous frame. This way, the pipeline does not need to scan the entire image, and the pixels detected have a high confidence of belonging to the lane line because they are based on the location of the lane in the previous frame. 

If at any time, the pipeline fails to detect lane pixels based on the the previous frame, it will go back in to blind search mode and scan the entire binary image for nonzero pixels to represent the lanes.

In order to make the output smooth I chose to average the coefficients of the polynomials for each lane line over a span of 10 frames. The gif below is the result of my pipeline running on the test video provided for the project, as well as an  optional challenge video which presented additional challenges to the lane detection pipeline.

|Project Video|Challenge Video|
|-------------|-------------|
|![Final Result Gif](./images/project_vid.gif)|![Challenge Video](./images/challenge.gif)|

### Possible Limitations:
The video pipeline developed in this project did a fairly robust job of detecting the lane lines in the test video provided for the project, which shows a road in basically ideal conditions, with fairly distinct lane lines, and on a clear day. It also did a decent job with the challenge video, although it did lose the lane lines momentarily when there was heavy shadow over the road from an overpass. 

What I have learned from this project is that it is relatively easy to finetune a software pipeline to work well for consistent road and weather conditions, but what is challenging is finding a single combination which produces the same quality result in any condition. I have not yet tested the pipeline on additional video streams which could challenge the pipeline with varying lighting and weather conditions, road quality, faded lane lines, and different types of driving like lane shifts, passing, and exiting a highway. For further research I plan to record some additional video streams of my own driving in various conditions and continue to refine my pipeline to work in more varied environments.   

<p align="center">
  <img src="./results/Model_Predictive_Control.gif" width="600">
</p>

## Results

The resulting [videos](./results/Model_Predictive_Control.webm) are in the repo, if you are interested. 

## Literature



## Contributing

No further updates nor contributions are requested.  This project is static.

## License

Term1_project1_lane_finding results are released under the [MIT License](./LICENSE)