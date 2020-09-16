# **Traffic Sign Recognition** 

## Build a Traffic Sign Recognition Project

### In this project and related lessons, I could learn how to use deep learning for image classification with tensorflow. I had used github codes provided with papers before, but this lessons were helpful in organizing the knowledge.

---

The goals / steps of this project are the following:
* Load the data set (see below for links to the project data set)
* Explore, summarize and visualize the data set
* Design, train and test a model architecture
* Use the model to make predictions on new images
* Analyze the softmax probabilities of the new images
* Summarize the results with a written report


[//]: # (Image References)

[image1]: ./writeup_images/data_vis01.jpg "Visualization"
[image2]: ./writeup_images/sample_all.jpg "Images of all labels"
[image3]: ./writeup_images/sample19.jpg "Image set of label 19"
[image4]: ./writeup_images/sample01.jpg "Image set of label1"
[image5]: ./writeup_images/preprocess_gray02.jpg "preprocess of grayscale images"
[image6]: ./writeup_images/preprocess_col04.jpg "preprocess of color images"
[image7]: ./writeup_images/sample_from_web.jpg "sample images from the web"

## Rubric Points

### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/481/view) individually and describe how I addressed each point in my implementation.  here is a link to my [project code](https://github.com/udacity/CarND-Traffic-Sign-Classifier-Project/blob/master/Traffic_Sign_Classifier.ipynb)

---

## Data Set Summary & Exploration

#### 1. In  Traffic_Sign_Classifier.ipynb cell01 & cell02, I loaded the dataset provided and preformed a basic analysis. Followings are the results of the analysis.

* Number of training examples = 34799
* Number of validation examples = 4410
* Number of testing examples = 12630
* Image data shape = (32, 32, 3)
* Number of classes = 43
* Depth of images = 3

#### 2. Here is an exploratory visualization of the data set. It is a bar chart showing how the amount of data for each traffic sign varies.

![alt text][image1]

 In cell04, I worte the code to list the images for each traffic sign. An example is shown below. As you can see in the image, I have found that the quality of each image varies widely. 

![alt text][image2]

 In particular, the brightness of the images varied greatly, and the label 19, 20 and 23 included many images that were too dark to identify the signs.  Also, some images might have lost color information. For example, in the image group of label 1, red circle of the sign becomes yellowish, then the color of the sign looks different.  When examining the model architecture in the next section, it was strange to me that there was no significant difference in validation accuracy between using color images and using grayscale images.  This is because I considered color information to be important for identifying traffic signs. However, looking at the data of label 1 images, I felt that the color information could have positive or negative impact where the color tone could change easily due to light condition.

![alt text][image3]
![alt text][image4]


## Design and Test a Model Architecture

### 1. Data preprocess

 As data preprocessing, I tried normalization and grayscale conversion. I wrote the preprocessing code for grayscale in cell06 and the preprocessing code for color in cell07. Also, in cell08, I wrote the code to visualize preprocessing results. Initially, I used (pixel - 128)/ 128 given in the template as the normalization methods, but the validation accuracy could not be improved by that method. Of the trials and errors to improve the validation accuracy, the normalization method had the greatest impact. Actually, the validation accuracy was only about 5% with the above method. Therefore, when the method shown in cell06 and cell07 using the average and variance of the pixel values of the image, the validation accuracy was greatly improved.
 Here is an example of a traffic sign image before and after applying preprocessing.

![alt text][image5]
![alt text][image6]


#### 2. Model architecture 

My final model consisted of the following layers:

| Layer         		|     Description	        					| 
|:---------------------:|:---------------------------------------------:| 
| Input         		| 32x32x3 RGB image   							| 
| Convolution 3x3x8    	| 1x1 stride, valid padding, outputs 30x30x8 	|
| RELU					|												|
| dropout				| keep_prob 0.75								|
| Convolution 3x3x8    	| 1x1 stride, valid padding, outputs 28x28x8 	|
| RELU					|												|
| dropout				| keep_prob 0.75								|
| Max pooling	      	| 2x2 stride,  outputs 14x14x8  				|
| Convolution 3x3x24   	| 1x1 stride, valid padding, outputs 12x12x24 	|
| RELU					|												|
| dropout				| keep_prob 0.75								|
| Convolution 3x3x24   	| 1x1 stride, valid padding, outputs 10x10x24 	|
| RELU					|												|
| dropout				| keep_prob 0.75								|
| Max pooling	      	| 2x2 stride,  outputs 5x5x24    				|
| Fully connected		| outputs 120 									|
| RELU					|												|
| Fully connected		| outputs 84 									|
| RELU					|												|
| Fully connected		| outputs 43 									|
| Softmax				| etc.        									|
|						|												|
|						|												|
 
I started from leNet archtecture provided in section13, lesson37, which was in cell09. Then modified code was in cell10_6. The above model was selected by the approach of section4 "Approach to improve accuracy". However, it was the preprocessing, not the model modification, that had the greatest impact on accuracy.

#### 3. Training model

To train the model, I used an Adam optimizer with softmax cross-entropy loss function.I did not make any trial and error in this process, and used the same code provided in Section13, lesson37. These codes were written in cell11 to cell13.

#### 4. Approach to improve accuracy
The first model I chose was the LeNet architecture given in the lesson(section37, lesson37) with the simple normalization methods of (pixel - 128)/128. This method was disastrously unsuccessful with validation accuracy of 5%. Then I started to modify the model archtecture. Initially, I thought about increasing the convolution layers to make it deeper, but this did not contribute to the accuracy improvement at all. Next, I thought about changing the preprocessing. I changed the color image to gray scale, but this also had little effect on the accucary. Then I changed the normalization method to use the mean and variance for each image.And with this, the valucation accuracy had improved remarkably.

My final model results were:
* training set accuracy of of 0.999
* validation set accuracy of 0.969
* test set accuracy of 0.944

After finding that the normalization method was important, I re-examined the effects of the revious changes. As a result, it was found that conversion to grayscale and model changes did not significantly affect the results. An example of the comparison result of epoch 100 is shown below. 

| validation accuracy| color conversion | normalization |    model		   | 
|:------------------:|:----------------:|:-------------:|:----------------:| 
|  0.962 			 | grayscale		| average		| LeNet modified01 | 
|  0.954 			 | grayscale		| average		| LeNet original   | 
|  0.05* 			 | grayscale		| simple		| LeNet modified01 | 
|  0.05* 			 | grayscale		| simple		| LeNet original   | 
|:------------------:|:----------------:|:-------------:|:----------------:| 
|  0.961 			 | color    		| average		| LeNet modified01 | 
|  0.955 			 | color    		| average		| LeNet original   | 
|  0.05* 			 | color    		| simple		| LeNet modified01 | 
|  0.05* 			 | color    		| simple		| LeNet original   | 

Improved validation accuracy was observed due to changes in the LeNet architecture with the addition of a convolutional layer(only one convolutional layer was added to LeNet:modified01). However, the effect of the conversion to grayscale was unclear at this stage.I thought that color images contained more information abou traffic signs and therefore showed better validation accuracy than grayscale, but that did not happen. Rather, conversely, grayscale showed better accuracy in the tests with the new images in the next section.

After the avobe trial, I tried various models by changing the parameters. I changed the number of filters(filter depth), filtering method('SAME' or 'VALID'), addition of another convolutional filter, dropping of the fc layer, addition of dropout layer, etc.
When the number of filters was increased, the calculation time increased drastically because the number of parameters were increased. However, the inmprovement in the validation accuracy was small for that. Regargding the filtering method, I expected that the 'SAME' method would give better results because the information at the edge of the image is not lost. However, when I actually tried it, 'VALID' method gave better results. The addition of the convolution layer had greatly contributed to the improvement of the validation accuracy. Also, the removal of the fully connected layer reduced the validation accuracy. Finally, I added two convolution layers and made the architecture shown in Sec.2.

The addition of the dropout layer showed a slight improvement in validation accuracy.However, it is unclear whether the difference was significant because the convergence varied.Rather, the validation accuracy seemed to be unstable. The evaluation was performed with a batch size of 128 and 100 epochs, but it might have improved if the number of epochs was increased.

After improving the normalization of preprocess, no model showed underfitting. On the other hand, as the training accuracy quickly reached 1.00 for most of the models, there might have seen a tendency for overfitting. Nevertheless, the validation accuracy also improved eventually.

### Test a Model on New Images

#### 1. Test with new images found on the web
I have selected ten German traffic signs from the web. The first five images were relatively good quality images, and the remaining five were with poor quality images.Belows are the images I chose.

![alt text][image7] 

!!!
Of the poor quality images, the images with label 14, 11, 12 might be difficult to classify due to occlusion, damege to the sign, and reflection of sunlight. On the other hand, I thought that images with label 5 and 30 were relatively easy. 
The first image might be difficult to classify because ...

#### 2. Discuss the model's predictions

Here are the results of the prediction:

| Image				        |     Prediction	        					| 
|:-------------------------:|:---------------------------------------------:| 
| 25. Road Work      		| 25. Road Work    								| 
| 18. General Caution 		| 18. General Caution 							|
| 12. Priority Road			| 12. Priority Road								|
| 14. Stop Sign	      		| 14. Stop Sign					 				|
| 22. Bumpy Road			| 22. Bumpy Road     							|
|:-------------------------:|:---------------------------------------------:| 
| 14. Stop Sign      		| 01. 20 km/h   								| 
| 05. 80 km/h     			| 05. 80 km/h 									|
| 11. Right-of-way			| 11. Right-of-way								|
| 12. Priority Road  		| 12. Priority Road 			 				|
| 30. Beware of Ice			| 11. Right-of-way     							|


The model was able to correctly guess 5 out of the 5 samples in the good image group, but could correctly predict only 3 out of 5 samples in the low quality image group. This gives a total accuracy of 80% (100% for good quality image group, whereas 60% of low quality image group). 
This does not compare favorably to the accuracy on the test set, especially I am wondering why the model could not correctly predict the fifth image in the low quality goup.

The code for making predictions on my final model is located in the cell17 and cell18 of the Ipython notebook.

#### 3. Describe how certain the model is 

In Cell19, I calculated the model's top 5 softmax probabilities for each image to show the certainty of the prediction. As you can see from the resulting probabilities, the predictions of the high-quality group were almost certain with a probability close to 1.0. 

Regarding the result of the low quality group, the probability of the top 1 prediction was almost 1.0 for the three images for which correct prediction was made.On the other hand, the situation is quiate different for the two mis-classified images. The 6th image(14.Stop Sign), correct label was not included in the top 5 predictions, whereas the 10th image(30. Beware of Ice), top 1 prediction was made with certain probability(almost 1.0) which was not correct. Unfortunately, it seemed that this situation did not change even if the validation accuracy was improved by changing the model architecture. 


| Image				        | Prediction |   Probability (1st, 2nd, 3rd, 4th, 5th) 	| 
|:-------------------------:|:----------:|:-----------------------------------------:| 
| 25. Road Work      		|  ○   		| 1.00, 0.00, 0.00, 0.00, 0.00  			| 
| 18. General Caution 		|  ○   		| 1.00, 0.00, 0.00, 0.00, 0.00  			| 
| 12. Priority Road			|  ○   		| 1.00, 0.00, 0.00, 0.00, 0.00  			| 
| 14. Stop Sign	      		|  ○   		| 1.00, 0.00, 0.00, 0.00, 0.00  			| 
| 22. Bumpy Road			|  ○   		| 1.00, 0.00, 0.00, 0.00, 0.00  			| 
|:-------------------------:|:------------------------------------------------------:| 
| 14. Stop Sign      		|  ×   		| 0.38, 0.12, 0.12, 0.06, 0.06  			| 
| 05. 80 km/h     			|  ○   		| 1.00, 0.00, 0.00, 0.00, 0.00 				|
| 11. Right-of-way			|  ○   		| 1.00, 0.00, 0.00, 0.00, 0.00 				|
| 12. Priority Road  		|  ○   		| 1.00, 0.00, 0.00, 0.00, 0.00 				|
| 30. Beware of Ice			|  ×   		| 1.00, 0.00, 0.00, 0.00, 0.00				|


### (Optional) Visualizing the Neural Network (See Step 4 of the Ipython notebook for more details)
#### 1. Discuss the visual output of your trained network's feature maps. What characteristics did the neural network use to make classifications?


