# **Path Planning**

## Writeup

### After reviewing the lessons, I thhought I could understand the concept of the behavior planning.
However, once I started the project, I noticed that writing the code of planning was very difficult.
As the project Q&A section was really helpful to start, I followed the outlines explained in the Q&A and incorpolate most of the Aaron's code. After letting the car drive, checking the points that the car did not drive well, I added my own additional code to improve the driving.   

---

**Path Planning Project**

The goals / steps of this project is to complete main.cpp:
Following points are need to be improved.
* The start of the car is very slow.
* Lane change timings were not appropriate. The car sometimes hit to the other car on the targetted lane.

[//]: # (Image References)

[image1]: ./writeup/1_start_graph.jpg "start_graph"
[image2]: ./writeup/2_start_original.jpg "start_original"
[image3]: ./writeup/3_start_improved.jpg "start_improved"
[image4]: ./writeup/4_success01.jpg "success01"
[image5]: ./writeup/5_incident.jpg "incident"
[image4]: ./writeup/6_success02.jpg "success02"
[image4]: ./writeup/7_success03.jpg "success03"

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/1971/view). As criterias of the rubric were checked automatically, I describe how I coded and debugged each function.  

---
### 1. start of the car

In Arron's code, refference velocity was increased 0.224 when the velocity was less than 49.5. This worked well when the car speeded up again after slowing down to avoide incident. However, at the very start and the speed of the car was very slow, the Arron's code needed to be improved to speed up more quickly.     

I improved to make the car accelerate quickly.
To speed up the car under the restriction of jerk and accleration(max jerk of 10m/s^3, max acceralation of 10m/s^2 from the rubric), I acceralate the car with constant jerk of 9.5.
First, I calculated what the speed whould be(end_vel) as a result of the accelaration.

```
  double Jc = 9.5;
  int n_points = 50 - previous_path_x.size();
  double dt = n_points*0.02;
  double end_vel = Jc*dt*dt/2.0*2.24 + ref_vel;
```

Then, as far as the end_vel is less than the speed limit, I acceralate with constant jerk of Jc.

```
  if(end_vel < 49.5){
    for (int i = 1; i <= 50 - previous_path_x.size(); i++){
      dt = i*0.02;
      x_point = x_add_on + Jc*dt*dt*dt/6.0 + (ref_vel*dt)/2.24;
      y_point = s(x_point);
```
Latter part of the loop was the same as Arron's code, converting back to the original coordinate system and adding to the next_x/y_vals. Here, I made sure that the x_add_on was set to zero only once before the loop, and watched out for the conversion between mph and meter per second.

After a while, I have noticed if I acceralate the car if the end_vel was only slightly smaller than the max_vel, I could not slow down our ego car properly when there was another car on the same lane. Therefore, I changed the condition to acceralete to consider the status as follows.

```
from
  if(ref_vel < 49.5){...
to
  if(ref_vel < 40.0 && too_close == false){...}
```

Result of the improvement is shown as follows.
![alt text][image1]

Unfortunately, the expected acceleration only happened at the very beginning. That was because most of the end points from the previous path were taken over, and only a few points were used to accerate at each step. However, the effect of the improvement worked as follows.
![alt text][image2]
![alt text][image3]

### 2. lane change ::
As Arron's code showed us an example to change lane if there was a car in front of us on the same lane, I add simple logic to improve it. First I checked if lane change to the left or right was possible. That means changing to the right lane was possible if my car was on the lane 0 or 1, whereas changing to the left lane was possible if my car was on the lane 1 or 2.

Then, I calculated the cost of change lane. The cost was set to be inversely proportional to the distance bewteen our car and the target car. As it was also dangerous if other cars existed on the target lane even if their position was behind me, I set the same const to the cars behind us but with smaller weight.

```lane change
  check_car_s += ((double)prev_size*0.02*check_speed);
  if((check_car_s > car_s) && ((check_car_s-car_s) < 60)){
    r_cost += 10.0/(check_car_s - car_s);
  }
  if((check_car_s <= car_s) && ((car_s - check_car_s) < 60)){
    r_cost += 5.0/(car_s - check_car_s);
  }
```
Then, I chose to change lane if the lane was available and the calculated cost was less than 1.0. Otherwise, slow down the car and wait for another chance.

```lane change
  if(r_cand != -1 && l_cand != -1){
    if(r_cost < l_cost && r_cost < 1.0){
      lane = r_cand;
    }else if(l_cost < r_cost && l_cost < 1.0){
      lane = l_cand;
    }
  }else if(l_cand != -1 && l_cost < 1.0){
    lane = l_cand;
  }else if(r_cand != -1 && r_cost < 1.0){
    lane = r_cand;
  }else{
    ref_vel -= 0.224;
  }
```

![alt text][image4]
With the above modifications, the car is able to drive more than 4.32 miles without incident.

#### 2.1. incident
However, I found the car sometimes hit another car in front of it. That happened when the car in front was slow, but there were other cars in the next lanes and the ego-car could not change lanes.   

![alt text][image5]

While observing the incidents, I noticed that the above collisions always occurred when the car was in the center lane. Then it turned out that there was a mistake in the above logic in which the ego-car was not able to decelerate properly when it was in the central lane even though slow car was in front and both side lanes were occupied. The modified algorithm is shown below.

```lane change
if(r_cand != -1 && l_cand != -1){
  if(r_cost < l_cost && r_cost < lane_change_thresh){
    lane = r_cand;
  }else if(l_cost <= r_cost && l_cost < lane_change_thresh){
    lane = l_cand;
  }else{
    ref_vel -= 0.224;
  }
}else if(l_cand != -1 && l_cost < lane_change_thresh){
  lane = l_cand;
}else if(r_cand != -1 && r_cost < lane_change_thresh){
  lane = r_cand;
}else{
  ref_vel -= 0.224;
}
```

After the above correction, the ego-car did not collide into the in front and it could drive longer without any violations.

![alt text][image6]

However, I think the ego-car changed lanes very frequently, and its driving looked dangerous. In fact, rubric conditions were violated during dangerous lane changes or two consecutive lane changes.
Therefore, I reduced the threashold (lane_change_thresh) to make it harder to change lanes.
As a result, the ego car was able to drive stably for a long time, even though the overall speed had slowed down slightly.

![alt text][image7]

### summary
With all of the above implementations, the ego car was able to drive stably for a long period of time while making appropriate lane changes.

Also I was upset when I noticed that my GPU time was running out. Thank you very much for increasing my GPU time quickly responding to my request.

![alt text][image4]
