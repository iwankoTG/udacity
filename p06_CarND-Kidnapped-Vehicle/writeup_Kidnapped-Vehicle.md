# **Kidnapped Vehicle**

## Writeup

### I thought particle filter was easier compared to Bayesian filter when I was studying the lesson. However, this project was hard at first and I noticed that there were a lot of things that I still had not understand. I think my understanding was deepened through debugging of the source code.

---

**Kidnapped Vehicle Project**

The goals / steps of this project is to complete particle_filter.cpp:
Following functions are need to be completed.
* init :: initialize all particles to first position
* prediction :: add measurements to each particle
* dataAssociation :: associate the predicted measurement and observed measurement
* updateWeights :: Update the weights of each particle
* resample :: resample particles with probability proportional to their weight


[//]: # (Image References)

[image1]: ./writeup/1_init_01.jpg "out of time error"
[image2]: ./writeup/2_pred_01.jpg "vehicle stopped"
[image3]: ./writeup/2_pred_02.jpg "vehicle move away"
[image4]: ./writeup/6_success_01.jpg "successfully finished"

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/747/view). As criterias of the rubric were checked automatically, I describe how I coded and debugged each function.  

---
### 1. init :: initialize all particles to first position

I set the number of particles and initialized their position with a Gaussian distribution. I followed the instructions in Section4 through 6 of Lesson 5.
First, I set the number of particles to 100. After debugging, I changed the value and examined the effects.
Below is a summary of my examination. I was surprized that a small number of particles could track the movement of the vehicle without significantly compromising accuracy. However, I think that many particles will be needed as the environment around the vehicle becomes more complex.  

|  number of particles |   success   |     error         |
|:--------------------:|:-----------:|:-----------------:|
|         1000         | out of time | xy:0.10 yaw:0.003 |
|          500         | out of time | xy:0.11 yaw:0.003 |
|          200         |   success   | xy:0.12 yaw:0.003 |
|          100         |   success   | xy:0.12 yaw:0.003 |
|           50         |   success   | xy:0.12 yaw:0.004 |
|           30         |   success   | xy:0.15 yaw:0.005 |
|           10         |   success   | xy:0.15 yaw:0.005 |

![alt text][image1]

### 2. prediction :: add measurements to each particle
In this function, I updated the position of each particle using the velocity and yaw_rate measurements. I followed the instructions in Section 7 through Section 9 of Lesson 5.
However, this was the last bug I needed to fix, but the equations given in the secsions above could only be used if yaw_rate was non-zero. When the absolute value of yaw_rate was less than 1.0E-08, I used the formula given in Section 4 of Lesson 3. Without this fix, the weights of particles became nan and the vehicle stopped suddenly. Moreover, if only the zero yaw_rate version of the equations were used, the vehicle left the route. Therefore, it was very important to branch the process by the absolute value of yaw_rate.

![alt text][image2]
![alt text][image3]

### 3. dataAssociation :: associate the predicted measurement and observed measurement
In this function, I searched the right correspondance between measurements in the prediction lists and those in the observation list. Following the explanation in Section 10 of Lesson 5, I used nearest neighbor technique. In other words, for each measurements in the observation lists, I selected closest landmarks from the predictin list.

### 4. updateWeights :: Update the weights of each particle
This function was the most difficult and complex function in the project. I divided it in two steps. The first step was to associate the observed measurements with the landmarks and the second step was to calculate the  likelihood of each particle. The likelihood of the particles became higher if the landmark positions from them were similar to the observed measurements. Therefore, by setting the likelihood to the particle weight, I could increase the weight of the particles that are likely to be near the vehicle.

#### 4.1. association
In order to associate the observed landmark measurements with the actual landmark position, I converted the landmark positions to the vehicle coordinate for each particle. I referred Section 15 in Lesson 5. Then, I used the dataAssociation function for the association.

#### 4.2. likelyhood calculation
It was proposed to use a mulivariate Gaussian distribution to calculate the likelihood. However, because the off-diagonal elements of the covariance matrix were zero(sigma_xy = 0, sigma_xx = sigma_yy = 0.3), the multivariate Gaussian distribution was the product of the univarite Gaussian distributions in the x and y directions.I referred quiz code of Section 25 of Lesson 2.

In addition, this time, the observation measurements given in vehicle coordinate were converted to map coordinate. I followed instructions given in Section 14 to 17 of Lesson 5 for the transformation.

#### 4.3. other
I added the procesures of 4.1 while I was debugging, but I feel that the coordinate transformation performed twice was redundant. It might have been better to use the dataAssociation function in map coordinate to avoid two transformations.

### 5. resample :: resample particles with probability proportional to their weights
This function resampled the particles based on the weights calculated by the updateWeights function. As weight of each particle reflected how close its position is to the vehicle, I could select more particles that were closer to the vehicle. I followed Sebastian's lecture in Section 18 through 20 of Lesson 4. In particular, I converted the quiz code in Section 20 to C++.

### summary
With all of the above implementations, I was able to localize the vehicle by correctly using the particle  filter.
![alt text][image4]
