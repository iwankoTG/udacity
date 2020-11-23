# **Capstone**

### Writeup
The purpose of this project is to write code to navigate Carla around the test track using the ROS system.
Based on the four workthroughs of the project, I could make the vehicle go around the simulator track using the information of traffic signal directory from the simulator. After that, I modified them to get the traffic signal information from the camera image using the created classifier.
---

[//]: # (Image References)

[image1]: ./writeup/1_drive_along_the_track.jpg "along_track"
[image2]: ./writeup/2_stop_at_traffic_lights.jpg "stop_lights"
[image3]: ./writeup/3_ssd_0.8_0.6.jpg "ssd"
[image4]: ./writeup/4_rfcn_0.8_0.6.jpg "rfcn"
[image5]: ./writeup/5_frcn_0.8_0.6.jpg "frcn"
[image6]: ./writeup/6_ssd_sim01_yel_0.6_0.3.jpg "frcn"
[image7]: ./writeup/7_ssd_sim05_0.6_0.3.jpg "frcn"
[image8]: ./writeup/8_stop_at_traffic_lights.jpg "frcn"
[image9]: ./writeup/9_final_goal.jpg "frcn"

### Rubric Points
Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/1969/view).

---
### 1. Make the vehicle go around the simulator track using /vehicle/traffic_lights
Even after implementing all the codes explained in the four walkthroughs, the vehicle did not go around the track with properly following the traffic lights. I fixed following three points.

#### 1.1 Correct waypoints_cb of tl_detector
Even thouth it was not mentioned in the walkthrough of tl_detector.py(Sec11), it was necessary to define waypoints_tree and waypoints_2d in tl_detector.py, similar to waypoint_updater.py. Therefore, I corrected waypoints_cb of tl_detector.py as follows.

```
def waypoints_cb(self, waypoints):
    self.waypoints = waypoints
    if not self.waypoints_2d:
        self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
        self.waypoint_tree = KDTree(self.waypoints_2d)
```

#### 1.2 Define loop function in tl_detector.py as in waypoint_updater.py
Similary, loop function was necessary in the tl_detector.py to control the frequency of function calls.
Prior to this correction, the light.state response was so slow(about 10 second late!) that the vehicle could not stop at the traffic light. Therefore, I changed rospy.spin() to self.loop() at the end of the init function of the TLDetector class and added the following loop function.

```
def loop(self):
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        if self.pose and self.waypoints:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        rate.sleep()
```

#### 1.3 Rate change
In the walkthrough, the rate of rospy.Rate(50) was given as 50. As Steven and Aaron said that 50Hz was too fast, I changed it from 50Hz to 30Hz. It worked when I was not using the camera images, but when I put on the camera, the vehicle started shaking and moving from the waypoints. Therefore, I changed the rate from 30Hz to 10Hz. As a result, the vehicle moved properly when the camera was turned on and stopped properly at the red light.

![alt text][image1]
![alt text][image2]


### 2. Make Classifier
Since Sec10 provided pre-trained models, I planned to use those models to detect traffic lights. After the detection, using the color information in the detected area, I planned to classify them in red, yellow and green.

#### 2.1 Traffic light detection
I compared the processing speed and accuracy and selected the optimal pre-trained model from the ssd, rfcn, faster rcnn provided in sec10.

|  number of particles |   speed     |   accuracy  |
|:--------------------:|:-----------:|:-----------:|
|          ssd         |   0.8 sec   |   low       |
|          rfcn        |   3.8 sec   |   normal    |
|       faster rcnn    |  16.0 sec   |   high      |

![alt text][image3]
![alt text][image4]
![alt text][image5]

The detection accuracy of the ssd was not high, but I thought it was suitable as a classifier this time because of its high processing speed.

For simulator images, adjusting confidence_cutoff will allow the ssd to detect traffic lights. Since there are few confusing objects in the simulator, I decided to use a low confidence_cutoff.

![alt text][image6]
![alt text][image7]

#### 2.2 Classification
After acquiring the area of the traffic light, the area was vertically divided into three areas, and the signal was identified by detecting the color of each area.
That part of the algorithm was described in the get_traffic_light_state function of tl_classifier.py.
I calculated the average of each color in the region, and if the red intensity was stronger than the others, I determined that the red signal was on.

```
box_red = image[int(bot):int(bot+box_height),int(left):int(right)]
val = np.mean(box_red[:,:,2])/(np.mean(box_red[:,:,0]) + np.mean(box_red[:,:,1]) + np.mean(box_red[:,:,2]))
if(val > 0.5): state_red +=1
```

#### 2.3 Others
In order to drive the vehicle smoothly, it was important to reduce the frequency of image processing. Therefore, I changed the logic to call process_traffic_lights only when the vehicle was less than 200 to the nearest traffic light waypoint. Without this modification, the car was off the waypoints, even if there were no obstacles around the car.  

```
if closest_light and line_wp_idx - car_wp_idx < 200:
   state = self.get_light_state(closest_light)
   return line_wp_idx, state
```

### summary
With all the above implementations, the vehicle was able to go around the simulator track while properly following the traffic lights.

![alt text][image8]
![alt text][image9]

However, as I have made a lot of manual adjustment to get the classifier to work in the simulator, I'm afraid my code was not robust enough for Carla to drive safely.
I wanted to fine-tune the ssd with transfer learning, but I could not do it on time. I will contine to try it out, so I would be appreciated your feedback and advice for transfer learning of objectdetection, especially with tensorflow.keras.
