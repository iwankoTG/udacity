# **Behavioral Cloning** 

## Summary

### The hardest part of this project was collecting good training data. Above all, my driving skill was low and I had a hard time creating data to teach the model. I also realized that skills such as data augumentation were less effective if the quality of the original data was low. Although I have heard that the quality of data was essential, I was able to realize it through my driving skills.

---

**Behavioral Cloning Project**

The goals / steps of this project are the following:
* Use the simulator to collect data of good driving behavior
* Build, a convolution neural network in Keras that predicts steering angles from images
* Train and validate the model with a training and validation set
* Test that the model successfully drives around track one without leaving the road
* Summarize the results with a written report


[//]: # (Image References)

[image1]: ./examples/center_2020_08_02_05_04_54_648.jpg "center lane"
[image2]: ./examples/center_2020_08_02_05_04_32_355.jpg "off the track01"
[image3]: ./examples/center_2020_08_02_05_10_56_828.jpg "off the track02"

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/432/view) individually and describe how I addressed each point in my implementation.  

---
### Files Submitted 

#### My project includes the following files:
* model_nvidia_wrc.py containing the script to create and train the model
  also model_lenet1wrc.py and model_lenet2wrc.py were the scripts for lenet models
* drive.py for driving the car in autonomous mode(not modified)
* model_NVIDIA_wdo2_0.5.h5 containing a trained convolution neural network 
* writeup_Behavioral_Cloning.md or writeup_report.pdf summarizing the results

### Model Architecture and Training Strategy

#### 1. An appropriate model architecture has been employed
In this project, I used three models to train the data. The original LeNet shown in Section14, modified LeNet I explored in the last project, and the NVIDIA model shown in Section15. Compared to the LeNet models, the NVIDIA's model was smaller in size, which I think was advantageous for edge use in autonomous driving.

model1:original leNet (total param :: 1,935,161)
       (model_lenet1wrc.py)

| Layer         		|     Description	        	| 
|:---------------------:|:-----------------------------:| 
| Input         		| 160x320x3 RGB image   		| 
| Lambda            	| normalization : x/255 - 0.5  	|
| cropping2d			| 65x320x3 RGB image			|
| Convolution 5x5x6    	| with RELU						|
| dropout				| keep_prob 0.75				|(added later)
| Max pooling	      	| 2x2 stride 					|
| Convolution 5x5x16   	| with RELU						|
| dropout				| keep_prob 0.75				|(added later)
| Max pooling	      	| 2x2 stride					|
| Fully connected		| outputs 400 					|
| Fully connected		| outputs 120 					|
| Fully connected		| outputs 84 					|
| Fully connected		| outputs 1 					|
|						|								|


model2:modified leNet (total param :: 6,468,867)
       (model_lenet2wrc.py)

| Layer         		|     Description	        	| 
|:---------------------:|:-----------------------------:| 
| Input         		| 160x320x3 RGB image   		| 
| Lambda            	| normalization : x/255 - 0.5  	|
| cropping2d			| 65x320x3 RGB image			|
| Convolution 3x3x6    	| with RELU						|
| dropout				| keep_prob 0.75				|(added later)
| Convolution 3x3x6    	| with RELU						|
| dropout				| keep_prob 0.75				|(added later)
| Max pooling	      	| 2x2 stride 					|
| Convolution 3x3x16   	| with RELU						|
| dropout				| keep_prob 0.75				|(added later)
| Convolution 3x3x16   	| with RELU						|
| dropout				| keep_prob 0.75				|(added later)
| Max pooling	      	| 2x2 stride					|
| Fully connected		| outputs 400 					|
| Fully connected		| outputs 120 					|
| Fully connected		| outputs 84 					|
| Fully connected		| outputs 1 					|
|						|								|

model3:NVIDIA (total param :: 348,219)
       (model_nvidia_wrc.py)

| Layer         		|     Description	        	| 
|:---------------------:|:-----------------------------:| 
| Input         		| 160x320x3 RGB image   		| 
| Lambda            	| normalization : x/255 - 0.5  	|
| cropping2d			| 65x320x3 RGB image			|
| Convolution 5x5x24   	| with RELU						|
| Convolution 5x5x36   	| with RELU						|
| Convolution 5x5x48   	| with RELU						|
| dropout				| keep_prob 0.5					|(added later)
| Convolution 3x3x64   	| with RELU						|
| Convolution 3x3x64   	| with RELU						|
| dropout				| keep_prob 0.5					|(added later)
| Fully connected		| outputs 100 					|
| Fully connected		| outputs 50 					|
| Fully connected		| outputs 10 					|
| Fully connected		| outputs 1 					|
|						|								|


#### 2. Attempts to reduce overfitting in the model

To combat the overfitting, I changed batch_size from 32 to 64 and 128. Batch size of 128 was better, but it took more time to converge.
I also addd dropout layers in order to reduce overfitting. However, the influence of the dropout layer was uncertain.

The model was trained and validated on different data sets to ensure that the model was not overfitting. The model was tested by running it through the simulator and ensuring that the vehicle could stay on the track.

#### 3. Model parameter tuning

For model1 and model3, I did not change the architecture and used the models as shown in the section. Also,the models used an adam optimizer, so the learning rate was not tuned manually (model.py line 25). For model2, I made small changes from the best scored architecture in Project 3.This is to reduce the model size.

#### 4. Appropriate training data

I used three different datasets to train the models. Data1 was the dataset provided as a project resource. Data2 and data3 were the datasets that I obtained.They were chosen to keep the vehicle driving on the road. I tried to drive on the center lane, but in data2 there were parts that were off the lane. As trainings were not successful, data3 was re-acquired. 

Also I used most of the preprocessing methods shown in the lessons, such as normalization with lambda, cropping, flipping and using other camera images. In particular, the method of adding images from the left and right cameras was very effective for autonomous driving. As this method did not improve validation accuracies, I did not understand the importance of it at first. However,when moving the vehicle in autonomous mode, the results changed significantly with and without this method.

### Model Architecture and Training Strategy

#### 1. Solution Design Approach

The overall strategy for deriving a model architecture was to compare the actual result of autonomous driving. At first, I looked at the validasion accuracy, but I also found that the accuracy does not always relate to the driving skill. Of course, there was a possibility that the models that could drive without leaving the lane were overfitting the course. However, this time, I judged the goodness of the models and methods based on the driving skills under the autonomous mode.

My first step was to use the original LeNet model and trained with dataset1. I thought this model might be appropriate because it worked well in project3.
Then adding improvement tips shown in the lessons. 

I knew image normalizatin was very important to classify images in project3. Then, I also knew that simple normalization such as pixel/255 - 0.5 shown in section10 did not work in the previous project. However, I decided to use that simple normalization this time, because the image brightness was constant in the current project. This means that this model could not be used for driving in different light conditions such as at night. I need to consider this point as future improvement.

I also added clipping and flipping processing. I thought that the clipping reduced the calculation load and was effective. I thought that flipping was also effective. The generalization performance might have improved if the data was acquired by running the course in the opposite direction. However, since my driving skill was low and data acquisition was difficult, I used flipping.

Then I added left and right camera images by following the suggestion of section13. As this process tripled the amount of data, it significantly increased the learning time. In addition, this process worsened the validation accuracy. Though it was natural as the amount of data increased, I thought this process was ineffective and removed it.  I later realized that this decision was wrong.

Below is the result of applying the model to data1 and data2.
Here, ○ indicates that the vehicle could drive in the autonomous mode without no tire's leaving the drivable portion of the track surface.△ indicates that the vehicle completed one round but it popped up onto ledges, and × indicates that the vehicle was off the course.
For each dataset, 'center only' indicates the case where only the center camera images were used, and 'with right' indicates the case where the images from right camera were also used. Here, I used only right camera images because of the memory restriction.
In data2, the dataset itself contained the part that the tires of the vehicle popped up onto ledges. In order to remove the adverse effect of it, I deleted the data of the corresponding part and trained the LeNet original model again, which made the result from ×→△.

Thanks to the use of multiple cameras and good training data, we were able to correctly train autonomous driving models regardless of the model.

| Model         		|     data1	 　　       	|     data2	 　　       	|      data3	 　　    	|
|:---------------------:|:-----------------------:|:-----------------------:|:-----------------------:|
|			    		| center only|with right　| center only|with right　| center only|with right　| 
|:---------------------:|:-----------------------:|:-----------------------:|:-----------------------:|
| LeNet original   		| ×			| △			| ×			| ×->△     | ×		   | ○		   |
| LeNet modified       	| ×			| ×			| ×			| ○			| ×			| ○			|
| Nvidia    			| ○			| ○			| ×			| ○			| ×			| ○			|
|:---------------------:|:-----------------------:|:-----------------------:|:-----------------------:|


However, the above results did not include the dropour layers. Contrary to my initial expectation, the addition of the dropout layer drastically reduced the accuracy of autonomous driving. I also tried adding dropout layers to the above condition of data3 and multiple camera. Unfortunately, except for the one condition of model3 (nvidia), all the vehicles were off the road. 

| Model         		| keep prob of 0.75 |  keep prob of 0.5  |   
|:---------------------:|:-----------------:|:------------------:|
| LeNet original   		| 		 ×		   | 		 ×		   |
| LeNet modified       	| 		 ×		   |		 ×		   | 
| Nvidia    			| 		 ×		   |		 ○		   |
|:---------------------:|:-----------------:|:------------------:|

With the dropout layers, I felt that there was less rolling on the left and right, and driving was more stable. However, the steering wheel operation was delayed at sharp curves, and the vheicle plunged into the pond.To compare the results, I included the resulting video and .h5 file in without_dropout folder. 

#### 2. Final Model Architecture

Based on the results so far, I chose model3 with dropout as the final architecture. There was no noticeable difference in the accuracy of autonomous driving, so I chose this model because of its small model size.

model3:NVIDIA (total param :: 348,219)
 (model_nvidia_wrc.py)

| Layer         		|     Description	        	| 
|:---------------------:|:-----------------------------:| 
| Input         		| 160x320x3 RGB image   		| 
| Lambda            	| normalization : x/255 - 0.5  	|
| cropping2d			| 65x320x3 RGB image			|
| Convolution 5x5x24   	| with RELU						|
| Convolution 5x5x36   	| with RELU						|
| Convolution 5x5x48   	| with RELU						|
| dropout				| keep_prob 0.5					|
| Convolution 3x3x64   	| with RELU						|
| Convolution 3x3x64   	| with RELU						|
| dropout				| keep_prob 0.5					|
| Fully connected		| outputs 100 					|
| Fully connected		| outputs 50 					|
| Fully connected		| outputs 10 					|
| Fully connected		| outputs 1 					|
|						|								|

#### 3. Creation of the Training Set & Training Process

To capture good driving behavior, I first recorded one laps on track one using center lane driving. Here is an example image of center lane driving:

![alt text][image1]

However, as data acquisition was difficult, data2 had included the following off the course images.

![alt text][image2]
![alt text][image3]

Then I repeated this process in order to get more data points.
I finally randomly shuffled the data set and put 20% of the data into a validation set. 

