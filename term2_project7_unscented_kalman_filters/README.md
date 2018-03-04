# Unscented Kalman Filter

## Overview

Kalman Filter is a mathematical approach of determining the state of the system. It calculates the system's state using a mathematical model of the motion and clarifies the state using the measurement information. For an introduction to UKFs, please refer to the paper ["The Unscented Kalman Filter for Nonlinear Estimation"](https://www.seas.harvard.edu/courses/cs281/papers/unscented.pdf).
  
Sensor fusion - is a process of combining measurements from different sensors to get one accurate picture. Sensor Fusion for object tracking using RADAR and LIDAR sensors is an actual task for the self-driving car. 

![Kalman_Filters_process_flow](./img/Kalman_Filters_process_flow.jpg)
Image: Udacity Self-Driving Car Nanodegree  


## Use Instructions

If you have already installed all the necessary dependencies for the projects in term 2 you should be good to go! If not, you should install them to get started on this project => [Getting Started for Term 2](../term2_How_to_get_started). 

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
   * On Windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./UnscentedKF`

Once you launched the executable, simply run the simulator app and select the EKF/UKF simulation.

## Goal of this project

In this project we implement an Unscented Kalman Filter (UKF) using the **constant turn rate and velocity magnitude model** (CTRV) motion model in C++. Noisy LIDAR and RADAR measurements detecting a bicycle that travels around your vehicle are provided like in the EKF project. Using the Unscented Kalman filter and the measurements you are able to track the bicycle's position and velocity. 

## General Process

All Kalmen filters have the same three main steps:

* **initializing** Kalman filter variables
* **predicting** where our object is going to be after a time step Δt
* **updating** where our object is based on sensor measurements

Then the prediction and update steps repeat themselves in a loop.

A standard Kalman filter can only handle linear equations. Both the extended Kalman filter and the unscented Kalman filter allow you to use non-linear equations. The difference between EKF and UKF is how they handle non-linear equations. But the basics are the same: initialize, predict, update.

To measure how well the Unscented Kalman filter performs, the root mean squared error comparing the Kalman filter results with the provided ground truth will  be calculated afterwards. The simulator provides the ground truth state of the object to be tracked and displays the calculated root mean squared error (RMSE).

## File structure

Files in the Github src Folder:
1. main.cpp: 
  * communicates with the Term 2 Simulator receiving data measurements
  * calls a function to run the Unscented Kalman filter
  * calls a function to calculate RMSE
2. UKF.cpp:
  * initializes the Unscented Kalman filter
  * defines the predict and update functions
  * calls the predict funciton
  * calls the update function for either the lidar or radar sensor measurement
3. tools.cpp
  * function to calculate RMSE

## Results

**Process Noise**
For the CTRV model, two parameters define the process noise:
  * \\(ddot{\sigma}^2\\)
  * σa2 representing longitudinal acceleration noise (you might see this referred to as linear acceleration) 
  * σψ¨2 representing yaw acceleration noise (this is also called angular acceleration)

In the project, both of these values need to be tuned. Different values have to be tested in order to get a working solution. Tuning will involve:
  * guessing appropriate parameter values
  * running the UKF filter
  * deciding if the results are good enough
  * tweaking the parameters and repeating the process


Compare your UKF and EKF project results. Both projects use the same data file. RMSE, especially for Vx and Vy should be lower for the UKF project than the EKF project. Why might that be?


Summarization of the most important properties of the UKF as a sensor fusion tool for self driving cars:

1. with the UKF, you will be able to take noisy measurement data as input and provide a smooth position and velocity estimation of dynamic objects around you, without introducing a delay!!
2. you can provide an estimation of the orientation and the yaw rate of other vehicles using sensors that can't even directly observe these things
3. the UKF also gives information on how precise the result is because it always provides a covariance matrix for every estimation
4. and you know that this covariance matrix is realistic if the UKF performs a consistency check.
The uncertainty of the estimation result is very important for self driving cars, because if the position of your leading vehicle is quite uncertain at some time, you better keep a little more distance.


###############

You have seen a similar plot before in the EKF project. We see here a bicycle that is first driving straight and then turning into a circle. The path of the bicycle 
is shown in blue. The green line is the sequence of all measurements we receive both from LIDAR and RADAR. As before, these measurements are quite noisy. The orange dots are the estimation
resource of the UKF fusing laser and radar measurements. Remember, the linear process model in the last project had problems following the turn. The CTR model we're using this time
follows the turn quite nicely, and still provides a smooth position estimate. You can play around with the process noise values, and make the estimation even smoother. Or force it to follow
the measurements quicker. When you change the process noise values, also make sure to check the consistency of your filter. This is how the consistency check of my filter looks like.
What you see here in orange are the NIS values of the three dimensional radar measurements. I also plotted the 95% line in blue. Just as expected, a small number if NIS values exceed this line.
If your NIS values look like this, you know you have set up a consistent filter. This is how the NIS values of the laser measurements look like. Be aware that the 95% line is at a different level in this case
because the laser measurement is a two dimensional vector. The project description will give you details about the exact criteria for a successful consistency check. Remember if the UKF is consistent,
it means it provides a realistic covariance metrics. The UKF also estimates the velocity of the bicycle of course. It can do it with or without radar. But if you compare these two options, you will find out
that the velocity estimate converges much faster if you use the radar too. Give it a try and switch on and off both sensors, and see how they contribute in different ways. What I find really impressive is
how precise the UKF can estimate the orientation of the bicycle. None of our sensors is able to directly observe the orientation, but we still get a precise estimate. Even the yaw rate can be estimated
providing useful results. For autonomous vehicles, the yaw rate of other vehicles is very important to know. Imagine another car starting to change lanes or a bicycle in front of you wants to do a left turn.
Hopefully, they would both signalize their intention but in the end the yaw rate is the ultimate indicator for such a behavior prediction. Let's summarize the three most important probabilities of the UKF as 
a centrifusion tool for self driving cars. Number one, the UKF, you will be able to take noisy measurement data as input and provide a smooth position and velocity estimation of dynamic objects around you, 
without introducing a delay. Number two, you can provide an estimation  f the orientation and the yaw rate of other vehicles using sensors that can't even directly observe these things. And number three, in addition to that,
the UKF also give information on how precise the result is. Because it always provides a covariance matrix for every estimation. And you know that this covariance matrix is realistic if the UKF performs a consistency check.
The uncertainty of the estimation result is very important for self driving cars. Because if the position of your leading vehicle is quite uncertain at some time, you better keep a little more distance

################






Lidar measurements are red circles, radar measurements are blue circles with an arrow pointing in the direction of the observed angle, and estimation markers are green triangles. The video below shows what the simulator looks like when a c++ script is using its Kalman filter to track the object. The simulator provides the script the measured data (either lidar or radar), and the script feeds back the measured estimation marker, and RMSE values from its Kalman filter. 

![Visualization_Extended_Kalman_Filters_overview](./results/Extended_Kalman_Filters_V02.jpg)


Using this submitted code for the first data set you will get the following vaules for RMSE:

||RMSE|
| :---:         |     :---:      |
|X | 0.0977 |
|Y | 0.0850 |
|Vx | 0.4025 |
|Vy | 0.4645 |

Plotting the lissajous figure using the visualization toolkit generates the following image.  This is specifically for the first data set:

![Visualization_Extended_Kalman_Filters](./results/Extended_Kalman_Filters_V01.jpg)

As you can see in the visualization, you can barely see the estimate since the ground truth nearly always covers it. The most variation is at the areas where the lissajous figure curves most rapidly. This is not surprising given the nature of the Extended Kalman Filter with the linear nature of its prediction.

<img src="./results/Extended_Kalman_Filters_V01.gif" width="980" alt="Combined Image" />


## Visualisation Tools

Visualization of the output is courtesy of the Jupyter visualization notebook 
[Sensor Fusion Toolkit](https://github.com/udacity/CarND-Mercedes-SF-Utilities) from the 
Mercedes team that accompanies the project.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Results

The resulting videos are in the repo, if you are interested.  

## Contributing

No further updates nor contributions are requested.  This project is static.

## License

Term1_project1_lane_finding results are released under the [MIT License](./LICENSE)