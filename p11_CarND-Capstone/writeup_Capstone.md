# **Capstone**

### Writeup

The purpose of this project is to write code to navigate Carla around the test track using ROS system.
Following the four workthroughs of the project. I could make the vehicle go around the simulator track using the information of traffic signal directory from the simulator.
Therefore, this writeup is about how to make classifier of the traffic light for Carle to judge the traffic signal from the camera image.
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
Even after implementing all the points suggested in the four walkthroughs, the vehicle did not go around the trach not stop at the red traffic lights. I fixed following three points.

#### 1.1 Correct waypoints_cb of tl_detector as that of waypoint_updater.py
Even thouth it was not mentioned in the walkthrough of tl_detector.py(Sec11), it was necessary to define waypoints_tree and waypoints_2d in tl_detector.py, similar to waypoint_updater.py. Therefore, I coorected waypoints_cb of tl_detector.py as follows.

```
def waypoints_cb(self, waypoints):
    self.waypoints = waypoints
    if not self.waypoints_2d:
        self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
        self.waypoint_tree = KDTree(self.waypoints_2d)
```

#### 1.2 define loop function in tl_detector.py as in waypoint_updater.py
Similary, loop function was necessary in the tl_detector.py to control the frequency of function call.
Before this correction, the vehicle could not stop at the traffic light at all because the response of light.state was so slow (about 10 second late!). Therefore, I changed rospy.spin() to self.loop() at the end of init function of the TLDetector class, and add following loop function.

```
def loop(self):
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        if self.pose and self.waypoints:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        rate.sleep()
```

###1.3 Rate change
In the walkthrough, a rate in rospy.Rate(50) was given as 50. As Steven and Aaron discussed that 50Hz was too fast. I changed it from 50 to 30Hz. It worked when I had not used camera images, but once I put the camera on, the vehicle started to wiggle and move out of the waypoints. When I changed the rate from 30Hz to 10Hz, the vehicle moved correctly even when the camera was turned on, and it stopped properly at the red traffic light.

![alt text][image1]
![alt text][image2]


### 2. Make Classifier
Next, I tried to add classifier. As pretrained models were provided in Sec10, I planned to use those pretrained models to detect traffic lights, and classify them in red, yellow and blue using color information within the detected region.

#### 2.1 Traffic light detection
I compared processing speed and accuracy to select the best pretrained model from ssd, rfcn, faster rcnn given in sec10.

|  number of particles |   speed     |   accuracy  |
|:--------------------:|:-----------:|:-----------:|
|          ssd         |   0.8 sec   |   low       |
|          rfcn        |   3.8 sec   |   normal    |
|       faster rcnn    |  16.0 sec   |   high      |

![alt text][image3]
![alt text][image4]
![alt text][image5]

The detection accuracy of ssd was not high, but I thought that it was suitable as a classifier this time because of its high processing speed.

For simulator images, ssd could detect traffic signals if I tuned confidence_cutoff. Since there are few confusing objects in the simulator, I decided to use it with the lower confidence_cutoff.

![alt text][image6]
![alt text][image7]

#### 2.2 Classification
After getting regions of traffic light, the region was divided into three areas in the vertical direction, and the signal was identified by detecting the color of each area.
That part of the algorithm was written at get_traffic_light_state function in tl_classifier.py.
I calculated mean of each color of the area, and if the red intensity was stronger than other, I decided that the red signal was on.

```
box_red = image[int(bot):int(bot+box_height),int(left):int(right)]
val = np.mean(box_red[:,:,2])/(np.mean(box_red[:,:,0]) + np.mean(box_red[:,:,1]) + np.mean(box_red[:,:,2]))
if(val > 0.5): state_red +=1
```

#### 2.3 Others
To drive the vehicle smoothly, it was important to reduce the frequency of image processing. Therefore, I changed the logic to call process_traffic_lights only when vehicle's position was less than 200 to the closest light waypoint. Without this modification, the car drive away to the waypoints when it seemed there were no obstacles around the car.  

```
if closest_light and line_wp_idx - car_wp_idx < 200:
   state = self.get_light_state(closest_light)
   return line_wp_idx, state
```

### summary
With all the above implementation, I could make the vehicle go around the simulator track with properly following the traffic light. However, as I made a lot of manual tuning to make the classifier works for the simulator, I'm afraid my code was not robust enough for Carla to drive safely.

![alt text][image8]
![alt text][image9]

The core probelm was the classifier. I should have fine tune the ssd pre-trained model to detect separately for each red, yellow, green signal. Or at least I wannted to make it detect more small traffic signals.
I tried to tune the ssd with transfer learning, but I could not make it on time. I'll contine to try, so I'd be grateful if you give me advice in the feedback.
