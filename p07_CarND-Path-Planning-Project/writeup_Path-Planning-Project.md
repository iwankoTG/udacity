# **Path Planning**

## Writeup

The purpose of this project is to enable ego-car to drive safely and smoothly on the highway.
The project Q&A section of David and Aaron was really helpful, so I followed the outlines explained in the Q&A and incorpolated most of the code described there. After making the car drive with that code, I checked where the car did not drive well and then modified the code to improve driving.   

---

**Path Planning Project**

The following points needed to be improved:
* The start of the car was very slow.
* The timing of the lane change was not appropriate. The ego-car could hit another car in the target lane.

[//]: # (Image References)

[image1]: ./writeup/1_start_graph.jpg "start_graph"
[image2]: ./writeup/2_start_original.jpg "start_original"
[image3]: ./writeup/3_start_improved.jpg "start_improved"
[image4]: ./writeup/4_success01.jpg "success01"
[image5]: ./writeup/5_incident.jpg "incident"
[image6]: ./writeup/6_success02.jpg "success02"
[image7]: ./writeup/7_success03.jpg "success03"

### Rubric Points
Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/1971/view). As the rubric criterias were checked automatically, I describe how I improved functions in main.cpp.  

---
### 1. start of the car

In Aaron's code, refference velocity was increased 0.224 when the velocity was less than 49.5. This worked well when the car accelerated again after slowing down to avoide incidents. However, at the very beginning the car was so slow that I had to improve Aaron's code to speed it up quickly.

To speed up the car under jerk and acceleration limits(max jerk of 10m/s^3, max acceralation of 10m/s^2 from the rubric), I acceralated the car with a constant jerk of 9.5.
First, I calculated the velocity(end_vel) as a result of the accelaration.

```
  double Jc = 9.5;
  int n_points = 50 - previous_path_x.size();
  double dt = n_points*0.02;
  double end_vel = Jc*dt*dt/2.0*2.24 + ref_vel;
```

Then, as long as end_vel was below the speed limit, I acceralated with a constant jerk(Jc).

```
  if(end_vel < 49.5){
    for (int i = 1; i <= 50 - previous_path_x.size(); i++){
      dt = i*0.02;
      x_point = x_add_on + Jc*dt*dt*dt/6.0 + (ref_vel*dt)/2.24;
      y_point = s(x_point);
```

The latter part of the loop was the same as Aaron's code, converting back to the original coordinate system and adding to the next_x/y_vals. Note that x_add_on was set to zero only once before the loop and that the conversion between mph and meter per second was done properly.

Then, I realized that under these conditions, the ego-car could not decelerate properly even if there was a slow car in front of it.Therefore, the conditions have been changed as follows.

```
from
  if(ref_vel < 49.5){...
to
  if(ref_vel < 40.0 && too_close == false){...}
```

The results of the improvement are as follows.
![alt text][image1]

Unfortunately, the expected acceleration only occurred at the very beginning. This was because most of the endpoints in the previous pass were taken over and only a few points were used to accerate at each step. However, the effect of the improvement worked as follows.

** original **
![alt text][image2]

** improved **
![alt text][image3]

### 2. lane change
Aaron and David gave me an example of changing lanes when there was a slower car on the same lane, so I add simple logic to improve it. First, I checked if it was possible to change lanes to the left or right. That is, if the ego-car was in lane 0 or 1, the car could change to the right lane, whereas if the ego-car was in lane 1 or 2, it could change to the left lane.

Next, I calculated the cost of changing lanes. The cost was set to be inversely proportional to the distance bewteen the ego-car and the target car. It was dangerous to have another car in the target lane even in the rear, so I set the same cost for the car behind with different weights.

```lane change
  check_car_s += ((double)prev_size*0.02*check_speed);
  if((check_car_s > car_s) && ((check_car_s-car_s) < 60)){
    r_cost += 10.0/(check_car_s - car_s);
  }
  if((check_car_s <= car_s) && ((car_s - check_car_s) < 60)){
    r_cost += 5.0/(car_s - check_car_s);
  }
```

Then, I changed the lane if the lane was available and the caucluated cost was less than 1.0,
otherwise slowed down the car.

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
With the above modifications, the car was able to drive more than 4.32 miles without any incidents.

#### 2.1. incident
However, the ego-car sometimes bumped into another car in front of it. It happened when the car in front was slow, but there were other cars in the other lanes and the ego-car could not change lanes.   

![alt text][image5]

While observing the incidents, I noticed that the above collisions always occurred when the car was in the center lane. Then I found that the above logic was incorrect. It turned out that when the ego-car was in the center lane, it could not decelerate properly when a slow car was in front and both lanes were occupied. The modified algorithm is shown below.

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

After the above correction, the ego-car did not collide with the car in front and was able to drive longer without any violations.

![alt text][image6]

However, the ego-car changed lanes frequently and was dangerous driving. In fact, rubric conditions were violated during dangerous lane changes or two consecutive lane changes.
Therefore, I reduced the threashold (lane_change_thresh) to make it harder to change lanes.
As a result, the ego-car was able to drive stably for a long time, even though the overall speed had slowed down slightly.

![alt text][image7]

### summary
With all of the above implementations, the ego car was able to drive stably for a long period of time while making appropriate lane changes.
