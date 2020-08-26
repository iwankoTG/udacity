# **Extended Karman Filter**

## Writeup

### Even though lessons were very helpful and well explained, I could not feel I could understand the Kalman Filter properly. Through this project I reviewed the contents, the equations and the codes in the practice section and tried to grasp the essense of the Extended Kalman Filter.

---

**Extended Karman Filter Project**

The goals / steps of this project are the following:
* Complete the codes provided in the src directory, and make sure it compiles properly
* Ensure that the accuracy measured by RMSE for the given data meets the critetia of the Rubric
* Follows the algorithm criteria described in the Rubric, and make sure it meets specifications
* Follows the code Efficiency criteria to avoid unnecessary calculations
* Summarize the results with a written report


[//]: # (Image References)

[image1]: ./writeup/p05_rmse_data1.jpg "RMSE of data1"
[image2]: ./writeup/p05_rmse_data2.png "RMSE of data2"

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/748/view) individually and describe how I addressed each point in my implementation.  

---
### 1. Complete the codes provided in the src directory, and make sure it compiles properly

My project includes the following files in src directory:
* main.cpp main file which connect simulator and kalman filter code
* FusionEKF.cpp/h controling lidar and radar fusion using extended kalman filter
* kalman_filter.cpp/h kalman filter code body which describe predict and update function
* tools.cpp/h describing RMSE and Jacobian calculation
* writeup_Extended_Kalman_Filter.md summarizing the results

### 2. Ensure that the accuracy measured by RMSE for the given data meets the critetia of the Rubric
I got the RMSE of px, py, vx, vy and made graphs of the change overtime. The final values of RMSE were (0.0973, 0.0855, 0.4513, 0.4399) for data1 and (0.0726, 0.0967, 0.4579, 0.4966) for data2. These values are less than the accuracy criteria given in the Rubric, which are (0.11, 0.11, 0.52, 0.52). 

![alt text][image1]
![alt text][image2]

### 3. Follows the algorithm criteria described in the Rubric, and make sure it meets specifications
I modified kalman_filter.cpp/h, FusionEKF.cpp/h and tools.cpp/h to meet the specifications written in Rubric.
Followings are corrections I made and lesson codes I used.

#### 3.1. tools.cpp
I added code to calculate RMSE with reference to lesson24, section 23 and 24. Also code to calculate Jacobian is added wirh reference to the same lesson, section 19 and 20.

#### 3.2. kalman_filter.cpp
I added functions corresponding to kalman filter steps, predict and update. As Laser measurement has a linear relationship with the state, kalman filter equations learned in lesson 21 are used. However, Rader measurement has a non-linear relationship, we have to use extended kalman filter equations, which were described in updateEKF function.

For Predict and Update functions, I used code learned in lesson24, section 7 and 8.
Only y = z - H_*x_ is different here in the UpdateEKF function. As the relationship between the state x_ and measurements z are no longer linear, the conversion of H_*x_ shold be calculated following the equations learned in lesson24, section18. Also, code to restrict the range of phi between -pi to pi was added.

#### 3.3 FusionEKF.cpp/h
I initialized H_laser_ and Hj_ in the constructor.
In the ProcessMeasurement function, I added code to manage kalman filter operation with laser and radar sensor fusion. At first measurement, the coveriance matrix P and the transition matrix F were initialized. The Coveriance matrix Q were created as 4x4 matrix.
After that, the state vector x_ was initialized according to the sensor type. In the case of a laser, the inputs were directly put into the state vector. On the other hand, in the case of rader, measurements were assigned to the state vector after converting the input from polar to cartesian coordinates.

Before the prediction step, I updated the state transition matrix F according to the new elapsed time, following what I learned in lesson24, section14. Also, with given noise values, I updated the process covariance matrix Q following the contents of the same section.

I called different update step depending on the sensor type. If the sensor was a laser, I used R_laser_ and H_laser_ defined in the constructor and called the Update function. However, if the sensor was a radar, I calculated the conversion matrix H using Jacobian, and then called UpdateEKF function.


### 4. Follows the code Efficiency criteria to avoid unnecessary calculations
Below are some points that I have made to make the code more efficient.
1. I decleared noise values noise_ax and noise_ay in the constructor of FusionEKF, not in the ProcessMeasurement function.
2. I wrote code without using if sentence, where the range of phi was limited from -pi to pi.
