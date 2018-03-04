# Udacity Self-Driving Car Engineer Nanodegree 

![car2](./car2.jpg)  
Image: Udacity Self-Driving Car Nanodegree  

Learning to build the future, today!  

Self-driving cars represent one of the most significant advances in modern history. Their impact will go beyond technology, beyond transportation, beyond urban planning to change our daily lives in ways we have yet to imagine. Here are some considerations:

  - self-driving vehicles will save a lot of lives
  - they will make our lives also more comfortable (e.g. mobility for seniors) 
  - transport will be delivered as a service from companies who own fleets of self-driving vehicles 
  - transportation will become more tightly integrated and packaged into many services  
  - premium vehicle services will be available
  - beeing able to avoid crashes will change the vehicle body construction radically 
  - interior equipment will focus even more on comfort
  - emotion (max. speed, acceleration, handling, exterior design ..) might almost entirely leave transportation  
  - are parking lots or parking spaces in town centers necessary anymore?  
  - traffic flow will be better regulated
  - infrastructure utilization will be optimized
  - hugh amount of data will be collected and used
  - hacking of vehicles will be a serious issue
  - ...  

In this program you could learn the skills and techniques used by self-driving car teams at the most innovative companies in the world like NVIDIA, Mercedes-Benz, Uber ATG, Elektrobit. This amazing technology is practiced through interactive projects in computer vision, robotic controls, localization, path planning, machine learning and more.  

![car3](./car3.jpg)  
Image: Udacity Self-Driving Car Nanodegree  

In this repository I will share the **source code** of all the projects of **[Udacity Self-Driving Car Engineer Nanodegree](https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013)**.  

### Projects  

--- 
## Table of Contents

### Term 1: Computer Vision and Deep Learning

![term1](./term1.jpg)

----  

| [Getting Started for Term 1](./term1_How_to_get_started)       | [![T1P1](./term1_project1_lane_finding/results/white_giphy.gif)](https://drive.google.com/open?id=1fBMSefw1EDetCwyycY5tPYXKwe1AOi4H)<br>**Project 1:<br>Finding Lane Lines on the road**<br>[Code](./term1_project1_lane_finding)       | [![T1P2](./term1_project1_lane_finding/img/white_giphy.gif)](https://drive.google.com/open?id=1fBMSefw1EDetCwyycY5tPYXKwe1AOi4H)<br>**Project 2:<br>Traffic Sign Classifier**<br>[Code](./term1_project1_lane_finding)       |
| :---:         |     :---:      |          :---: |
| [![T1P3](./term1_project1_lane_finding/img/white_giphy.gif)](https://drive.google.com/open?id=1fBMSefw1EDetCwyycY5tPYXKwe1AOi4H)<br>**Project 3:<br>Behavioral Cloning**<br>[Code](./term1_project1_lane_finding)       | [![T1P4](./term1_project1_lane_finding/img/white_giphy.gif)](https://drive.google.com/open?id=1fBMSefw1EDetCwyycY5tPYXKwe1AOi4H)<br>**Project 4:<br>Advanced Lane Finding**<br>[Code](./term1_project1_lane_finding)       |[![T1P5](./term1_project1_lane_finding/img/white_giphy.gif)](https://drive.google.com/open?id=1fBMSefw1EDetCwyycY5tPYXKwe1AOi4H)<br>Project 5:<br>Vehicle Tracking <br>[Code](./term1_project1_lane_finding)|

---  

### _Introduction_  
The program starts with a survey of how self-driving cars work and an immediately set out to the very first autonomous vehicle project - finding lane lines on the road!   


#### [P1 - Finding Lane Lines on the road](term1_project1_lane_finding)
 - **Summary:** Detect lane lines on a highway within a video stream. Use OpencV image analysis techniques to identify lines of different colors.
 - **Keywords:** Python, Computer Vision, Hough Transforms, Canny edge detection
 
#### **Deep Learning**      
Deep learning has become the most important frontier in both machine learning and autonomous vehicle development. Experts from NVIDIA and Uber ATG teach to build deep neural networks and train them with data from the real world and from the Udacity simulator.  


#### [P2 - Traffic Sign Classification](term1_project2_traffic_sign_classifier)
 - **Summary:** Build and train a deep neural network to classify traffic signs, using TensorFlow. Experiment with different network architectures. Performe image pre-processing and validation to guard against overfitting.
 - **Keywords:** Python, Deep Learning, Convolutional Neural Networks, TensorFlow
 
#### [P3 - Behavioral Cloning](project_3_behavioral_cloning)
 - **Summary:** Build and train a convolutional neural network for end-to-end driving in a simulator, using TensorFlow and Keras. Use optimization techniques such as , data augmentation and dropout to generalize the network for driving on multiple tracks.
 - **Keywords:** Python, Deep Learning, Keras, Convolutional Neural Networks
 
#### **Computer Vision**  
Use a combination of cameras, software, and machine learning to find lane lines on difficult roads and to track vehicles. Start with calibrating cameras and manipulating images, and end by applying support vector machines and decision trees to extract information from video.  


#### [P4 - Advanced Lane Finding](project_4_advanced_lane_finding)
 - **Summary:** Write a software pipeline to identify the lane boundaries in a video from a front-facing camera on a car. Use distortion correction, image rectification, color transforms, and gradient thresholding. Identify lane curvature and vehicle displacement with means of perspective transform and polynomial fit. Overcame environmental challenges such as shadows and pavement changes.
 - **Keywords:** Python, OpenCV, Sobel Operator, Color Spaces  
 
#### [P5 - Vehicle Detection and Tracking](project_5_vehicle_detection)
 - **Summary:** Create a vehicle detection and tracking pipeline with OpenCV. Use techniques like 'histogram of oriented gradients' (HOG) and train a classifier (e.g. support vector machines (SVM)) to identify cars. Implement the model in a pipeline. Use sliding windows to detect cars on video data from an automotive camera taken during highway driving. Use heat-map technique to combine overlapping detections and remove false positives.
 - **Keywords:** Python, Object Detection and Tracking, SVM, HOG, Sliding Window, Heatmap  


 ---  

### Term 2: Sensor fusion and localization control

 ---  

![term2](./term2.jpg)

|[Getting Started for Term 2](./term2_How_to_get_started)        |[![T2P6](./term2_project6_extended_kalman_filters/results/Extended_Kalman_Filters_V01.gif)](https://drive.google.com/open?id=1fBMSefw1EDetCwyycY5tPYXKwe1AOi4H)<br>**Project 6:<br>Extended Kalman Filters**<br>[Code](./term2_project6_extended_kalman_filters)| [![T2P7](./term2_project7_unscented_kalman_filters/results/Unscented_Kalman_Filter.gif)](https://drive.google.com/open?id=1fBMSefw1EDetCwyycY5tPYXKwe1AOi4H)<br>**Project 7:<br>Unscented Kalman Filters**<br>[Code](./term2_project7_unscented_kalman_filters)|
| :---:         |     :---:      |          :---: |
|[![T2P8](./term2_project8_particle_filter_kidnapped_vehicle/results/particle_filter_kidnapped_vehicle.gif)](https://drive.google.com/open?id=1fBMSefw1EDetCwyycY5tPYXKwe1AOi4H)<br>**Project 8:<br>Particle Filer - Kidnapped Vehicle**<br>[Code](./term2_project8_particle_filter_kidnapped_vehicle)|[![T2P9](./term1_project1_lane_finding/img/white_giphy.gif)](https://drive.google.com/open?id=1fBMSefw1EDetCwyycY5tPYXKwe1AOi4H)<br>**Project 9:<br>PID Controller** <br>[Code](./term1_project1_lane_finding)|[![T2P10](./term1_project1_lane_finding/img/white_giphy.gif)](https://drive.google.com/open?id=1fBMSefw1EDetCwyycY5tPYXKwe1AOi4H)<br>**Project 10:<br>Model Predictive Control** <br>[Code](./term1_project1_lane_finding)|

---  


#### [P6 - Extended Kalman Filter](project_6_extended_kalman_filter)
 - **Summary:** Implement the extended Kalman filter in C++. Simulated lidar and radar measurements are used to detect a bicycle that travels around your vehicle. Use Kalman filter, lidar measurements and radar measurements  to track the bicycle's position and velocity.
 - **Keywords:** C++, Kalman Filter, CV motion model, Sensor fusion, Lidar, Radar

#### [P7 - Unscented Kalman Filter](project_7_unscented_kalman_filter)
 - **Summary:**  Utilize an Unscented Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Kalman filter, lidar measurements and radar measurements are used to track the bicycle's position and velocity.
 - **Keywords:** C++, Unscented Kalman Filter, CTRV motion model, Sigma Points, Consistency Check, NIS
 
#### [P8 - Kidnapped Vehicle](project_8_kidnapped_vehicle)
 - **Summary:** Your robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data. In this project you will implement a 2 dimensional particle filter in C++. Your particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step your filter will also get observation and control data.
 - **Keywords:** C++, Particle Filter
 
#### [P9 - PID Control](project_9_PID_control)
 - **Summary:** Implement a PID controller for keeping the car on track by appropriately adjusting the steering angle.
 - **Keywords:** C++, PID Controller
 
#### [P10 - MPC Control](project_10_MPC_control)
- **Summary:** Implement an MPC controller for keeping the car on track by appropriately adjusting the steering angle. Differently from previously implemented PID controller, MPC controller has the ability to anticipate future events and can take control actions accordingly. Indeed, future time steps are taking into account while optimizing current time slot.
- **Keywords:** C++, Model Preditive Controller (MPC)


---  

### Term 3: Path Planning Concentration

![term3](./term3.jpg)

---  

|[![T3P11](./term1_project1_lane_finding/img/white_giphy.gif)](https://drive.google.com/open?id=1fBMSefw1EDetCwyycY5tPYXKwe1AOi4H)<br>**Project 11:<br>Path Planning Project**<br>[Code](./term1_project1_lane_finding)|[![T3P12](./term1_project1_lane_finding/img/white_giphy.gif)](https://drive.google.com/open?id=1fBMSefw1EDetCwyycY5tPYXKwe1AOi4H)<br>**Project 12:<br>Advanced Deep Learning**<br>[Code](./term1_project1_lane_finding)|[![T3P13](./term1_project1_lane_finding/img/white_giphy.gif)](https://drive.google.com/open?id=1fBMSefw1EDetCwyycY5tPYXKwe1AOi4H)<br>**Project 13:<br>Functional Safety**<br>[Code](./term1_project1_lane_finding)|
| :---:         |     :---:      |          :---: |
|<br>**CAPSTONE PROJECT**<br>**System Integration**|[![T3P14](./term1_project1_lane_finding/img/white_giphy.gif)](https://drive.google.com/open?id=1fBMSefw1EDetCwyycY5tPYXKwe1AOi4H)<br>**Project 14:<br>Programming a Real Self-Driving Car**<br>[Code](./term3_capstone_project)||

---  

#### [P11 - Path Planning](project_11_path_planning)
- **Summary:** The goal in this project is to build a path planner that is able to create smooth, safe trajectories for the car to follow. The highway track has other vehicles, all going different speeds, but approximately obeying the 50 MPH speed limit. The car transmits its location, along with its sensor fusion data, which estimates the location of all the vehicles on the same side of the road.
- **Keywords:** C++, Path Planning

#### [P12 - Road Segmentation](project_12_road_segmentation)
- **Summary:** Implement the road segmentation using a fully-convolutional network.
- **Keywords:** Python, TensorFlow, Semantic Segmentation


#### [CAPSTONE PROJECT - Programming a Real Self-Driving Car](project_12_road_segmentation)
- **Summary:** Programm a real driving car.
- **Keywords:** C++, ROS, Semantic Segmentation, Path, Simulator & Real Car
 
<p align="center">
  <img src="https://cdn-images-1.medium.com/max/800/1*dRJ1tz6N3MqO1iCFzlhxZg.jpeg" width="400">
</p>
Image: Udacity Self-Driving Car Nanodegree  

## Contributing

No further updates nor contributions are requested.  This project is static.

## License

**UDACITY-self-driving-car** results are released under the [MIT License](./LICENSE).