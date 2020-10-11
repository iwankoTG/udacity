#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  //add code based on the instruction of project Q&A
  int lane=1;
  double ref_vel = 0.0; //mph

  h.onMessage([&ref_vel,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&lane]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          int prev_size = previous_path_x.size();

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           *   follow Aron's instruction in project Q&A
           */
          double max_vel = 49.5;
          bool prepare_for_change = false;
          
          //avoide collision
          if(prev_size > 0){
            car_s = end_path_s;
          }
          bool too_close = false;
          
          //check if there is a car ahead on the same lane
          for(int i = 0; i < sensor_fusion.size(); i++){
            float d = sensor_fusion[i][6];
            if(d < (2+4*lane + 2) && d > (2+4*lane-2)){
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx + vy*vy);
              double check_car_s = sensor_fusion[i][5];
              
              check_car_s += ((double)prev_size*0.02*check_speed);
              if((check_car_s > car_s) && ((check_car_s-car_s) < 30)){
                too_close = true;
              }
            }
          }
            
          //decide lane change or slow down
          if(too_close){
            //check if side-lanes are available
            int r_cand = -1;
            int l_cand = -1;
            double r_cost = 0;
            double l_cost = 0;
            if(lane == 0){
              r_cand = 1;
            }else if(lane == 1){
              r_cand = 2;
              l_cand = 0;
            }else{
              l_cand = 1;
            }
              
            //
            for(int i = 0; i < sensor_fusion.size(); i++){
              float d = sensor_fusion[i][6];
              if(r_cand != -1 && d < (2+4*r_cand + 2) && d > (2+4*r_cand-2)){
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx*vx + vy*vy);
                double check_car_s = sensor_fusion[i][5];
              
                check_car_s += ((double)prev_size*0.02*check_speed);
                if((check_car_s > car_s) && ((check_car_s-car_s) < 60)){
                  r_cost += 10.0/(check_car_s - car_s);
                }
                if((check_car_s <= car_s) && ((car_s - check_car_s) < 60)){
                  r_cost += 5.0/(car_s - check_car_s);
                }
              }
              if(l_cand != -1 && d < (2+4*l_cand + 2) && d > (2+4*l_cand-2)){
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx*vx + vy*vy);
                double check_car_s = sensor_fusion[i][5];
              
                check_car_s += ((double)prev_size*0.02*check_speed);
                if((check_car_s > car_s) && ((check_car_s-car_s) < 60)){
                  l_cost += 10.0/(check_car_s - car_s);
                }
                if((check_car_s <= car_s) && ((car_s - check_car_s) < 60)){
                  l_cost += 5.0/(car_s - check_car_s);
                }
              }
            }
            
            //decide if lane change are possible or not
            double lane_change_thresh = 0.8;
            std::cout << "too close" << std::endl;
            std::cout << l_cost <<", " << r_cost << std::endl;
            
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
                        
          }else if(ref_vel < 49.5){
            ref_vel += 0.224;
          }
                          
          vector<double> ptsx;
          vector<double> ptsy;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          //car is at the starting position
          if(prev_size < 2){
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }else{
            //use the previous path's end point at starting reference
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }
          
          //add evenly 30m spaced points ahead of the starting reference
          vector<double> next_wp0 = getXY(car_s+30,(2+4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60,(2+4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90,(2+4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          //shift car reference angle to 0 degrees
          for (int i=0; i<ptsx.size(); i++){
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;

            ptsx[i] = (shift_x*cos(-ref_yaw) - shift_y*sin(-ref_yaw));
            ptsy[i] = (shift_x*sin(-ref_yaw) + shift_y*cos(-ref_yaw));
          }

          //spline
          tk::spline s;
          //std::cout << ptsx.size() <<',' << ptsy.size() << std::endl;
          s.set_points(ptsx, ptsy);

          for(int i=0; i < previous_path_x.size(); i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          //
          double x_add_on = 0;
          
          //***improve point1:
          double keep_vel = 40.0;
          if(ref_vel < keep_vel && too_close == false){            
            //std::cout << "accelerating" << std::endl;
            double x_point;
            double y_point;
            double Jc = 9.5;
            int n_points = 50 - previous_path_x.size();
            double dt = n_points*0.02;
            double end_vel = Jc*dt*dt/2.0*2.24 + ref_vel;
            if(end_vel < keep_vel){
            std::cout << "accelerating:" <<ref_vel <<", " << end_vel << std::endl;
              for (int i = 1; i <= 50 - previous_path_x.size(); i++){
                dt = i*0.02;
                x_point = x_add_on + Jc*dt*dt*dt/6.0 + (ref_vel*dt)/2.24;
                y_point = s(x_point);
              
                double x_ref = x_point;
                double y_ref = y_point;

                x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
                y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));

                x_point += ref_x;
                y_point += ref_y;

                next_x_vals.push_back(x_point);
                next_y_vals.push_back(y_point);
              }
              ref_vel = end_vel;
            }
          }else{          
            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));
            for (int i = 1; i <= 50 - previous_path_x.size(); i++){
              double N = (target_dist/(.02*ref_vel/2.24));
              double x_point = x_add_on + target_x/N;
              double y_point = s(x_point);
            
              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
              y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
          }
          }

          //End of TODO
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}
