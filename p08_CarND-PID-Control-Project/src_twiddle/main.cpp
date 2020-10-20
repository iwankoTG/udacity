#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include <vector>
#include <float.h>
#include <time.h>

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid;
  /**
   * TODO: Initialize the pid variable.
   */
  //pid.Init(0.2, 0.004, 3.0); //P, I, D
  std::vector<double> p = {0.2, 0.004, 3.0};
  std::vector<double> dp = {0.02, 0.001, 0.3};
  std::vector<bool> upstate = {true, true, true};
  int n_turn = 50;
  int ind = 0;
  double best_err = DBL_MAX;
  double prev_err = DBL_MAX;
  p[ind] += dp[ind];
  pid.Init(p);
  clock_t start = clock();
  double ave_speed = 0.0;
  int ave_speed_count = 0;
  double base_throttle = 0.3;

  h.onMessage([&pid, &n_turn, &p, &dp, &ind, &best_err, &prev_err, &upstate, &start, &ave_speed, &ave_speed_count, &base_throttle]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();
        //std::cout << j[1] << std::endl;

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          ave_speed += speed;
          ave_speed_count += 1;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          int count = pid.UpdateError(cte);
          steer_value = pid.CalcSteering();
          //std::cout <<"count =" <<count << std::endl;
          
          if(cte > 4.0 || cte < -4.0){
            clock_t end = clock();
            const double duration = static_cast<double>(end - start) / CLOCKS_PER_SEC * 1000.0;
            std::cout << "OFF LANE: duration = " << duration <<"sec"<< std::endl;
            start = clock();
            ave_speed = 0.0;
            ave_speed_count = 0;
          }
          
          if(count == n_turn){
            //std::cout << "ind=" << ind <<", upstate[ind]" << upstate[ind] <<", p[ind]=" << p[ind] <<", dp[ind]=" <<dp[ind] <<std::endl;            
            //std::cout << "Kp=" << p[0] <<", Ki=" << p[1] <<", Kd=" << p[2] <<", best_err=" <<best_err <<std::endl;
            double err = pid.TotalError();
            if(upstate[ind] && err < prev_err){
              std::cout <<"area1" <<std::endl;
              best_err = err;
              dp[ind] *= 1.0;
              ind = (ind+1)%3;
            }else if(upstate[ind] && err >= prev_err){
              std::cout <<"area2" <<std::endl;
              p[ind] -= dp[ind];
              upstate[ind] = false;
            }else if(!upstate[ind] && err < prev_err){
              best_err = err;
              std::cout <<"area3" <<std::endl;
              dp[ind] *= 1.0;
              ind = (ind+1)%3;
            }else if(!upstate[ind] && err >= prev_err){
              std::cout <<"area4" <<std::endl;
              p[ind] += dp[ind];
              dp[ind] *= 0.9;
              upstate[ind] = true;
              ind = (ind+1)%3;
            }else{
              std::cout << "ERROR" << std::endl;
            }
            if(best_err > err){
              best_err = err;
            }
            if(upstate[ind] == true){
              p[ind] += dp[ind];         
              pid.Init(p);
              std::cout << "Kp=" << p[0] <<", Ki=" << p[1] <<", Kd=" << p[2] <<", best_err=" <<best_err <<", prev_err=" <<prev_err <<", err=" <<err<<std::endl;
              std::cout << "dKp=" << dp[0] <<", dKi=" << dp[1] <<", dKd=" << dp[2] <<", ind=" <<ind <<", cte=" <<cte <<std::endl;
            }else{
              p[ind] -= dp[ind];         
              if(p[ind] < 0.0){
                p[ind] = 0.0;
              }
              pid.Init(p);
              std::cout << "Kp=" << p[0] <<", Ki=" << p[1] <<", Kd=" << p[2] <<", best_err=" <<best_err <<", prev_err=" <<prev_err <<", err=" <<err<<std::endl;
              std::cout << "dKp=" << dp[0] <<", dKi=" << dp[1] <<", dKd=" << dp[2] <<", ind=" <<ind <<", cte=" <<cte <<std::endl;
            }
            prev_err = err;
            
            if(steer_value > 0.7 || steer_value < -0.7){
              base_throttle -= 0.01; 
            }
            else if(steer_value < 0.3 && steer_value > -0.3){
              base_throttle += 0.01; 
            }

            clock_t end = clock();
            const double duration = static_cast<double>(end - start) / CLOCKS_PER_SEC * 1000.0;
            double average = ave_speed/ave_speed_count;
            std::cout << duration << ", ave_speed(mph) = " << average <<", distance(mile) = " << duration*average/3600.0 <<", base_throttle = " << base_throttle << std::endl;
          }
          
          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          //msgJson["throttle"] = 0.3; 
          msgJson["throttle"] = base_throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
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