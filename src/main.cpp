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

  // initiate a lane number
  int lane = 1;
          
  // reference velocity to target
  double ref_vel = 0.0; //mph
  
  h.onMessage([&ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&lane]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
                
    int safety_buffer = 30; // m, used to measure distance for safety reason
                
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

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          int prev_size = previous_path_x.size();
          
          if (prev_size>0){
            car_s = end_path_s;
          }
          
          bool too_close = false;
          bool left_clear = true;
          bool right_clear = true;
          
          //find ref_vel to use
          for (int i=0; i<sensor_fusion.size(); i++){
//             bool too_close = false;
//             bool left_clear = true;
//             bool right_clear = true;
            //car is in my lane
            float d = sensor_fusion[i][6];
            
            // check left side, if there is closed vehicle
            if (lane!=0&&d<(4*lane)&&d>(4*lane-4)){
              double vx_left = sensor_fusion[i][3];
              double vy_left = sensor_fusion[i][4];
              double check_speed_left = sqrt(vx_left*vx_left + vy_left*vy_left);
              double check_car_s_left = sensor_fusion[i][5];
              check_car_s_left += ((double)prev_size*0.02*check_speed_left);
              
//               bool danger_for_change_left = (check_car_s_left<abs(car_s-safety_buffer/2)) && (check_car_s_left<abs(car_s+safety_buffer/2));
              bool danger_for_change_left = ((check_car_s_left < car_s && abs(check_car_s_left - car_s) < 30) || (check_car_s_left > car_s && abs(check_car_s_left - car_s) < 15));
//               if (((check_car_s_left < car_s)&&((-check_car_s_left+car_s)<safety_buffer))||((check_car_s_left > car_s)&&((check_car_s_left-car_s)<safety_buffer))){
              if (danger_for_change_left){
                left_clear = false;
              }
            }
            
            // check right side, if there is closed vehicle
            if (lane!=2&&d<(2+4*lane+6)&&d>(4+4*lane)){
              double vx_right = sensor_fusion[i][3];
              double vy_right = sensor_fusion[i][4];
              double check_speed_right = sqrt(vx_right*vx_right + vy_right*vy_right);
              double check_car_s_right = sensor_fusion[i][5];
              check_car_s_right += ((double)prev_size*0.02*check_speed_right);
              
//               bool danger_for_change_right = (check_car_s_right<abs(car_s-safety_buffer/2)) && (check_car_s_right<abs(car_s+safety_buffer/2));
              bool danger_for_change_right = ((check_car_s_right < car_s && abs(check_car_s_right - car_s) < 30) || (check_car_s_right > car_s && abs(check_car_s_right - car_s) < 15));
              
//               if (((check_car_s_right < car_s)&&((-check_car_s_right+car_s)<safety_buffer))||((check_car_s_right > car_s)&&((check_car_s_right-car_s)<safety_buffer))){
              if (danger_for_change_right){
                right_clear = false;
              }
            }
            
            // check if there is a closed car ahead
            if (d<(2+4*lane+2)&&d>(2+4*lane-2)){
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx + vy*vy);
              double check_car_s = sensor_fusion[i][5];
              check_car_s += ((double)prev_size*0.02*check_speed);
              
              if ((check_car_s > car_s)&&((check_car_s-car_s)<20)){
                too_close = true;
              } 
              
            }
            
          }
          
          if (too_close){
            ref_vel -=.284;
            if (left_clear&&lane!=0){
                  lane-=1; // change to the left lane
                }
                else if (right_clear&&lane!=2){
                  lane+=1; // change to the right lane
                }
          }
          else if (ref_vel<49.5){
            ref_vel +=.224;
          } // M: not understood completely
          
          std::cout << "lane: " << lane << "\ttoo close ahead: " << too_close << "\tfree on left: " << left_clear << "\tfree on right: " << right_clear << std::endl;
          
         // Creat a list of widely spread (x,y) waypoints, evenly spaced at 30m
          
          vector<double> ptsx;
          vector<double> ptsy;
          // reference x, y, yaw_rate
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          if (prev_size<2){
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          } else{
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
          
          //In Frenet add evenly 30m space points ahead of the starting reference
          vector<double> next_wp0 = getXY(car_s+30,(2+4*lane),map_waypoints_s, map_waypoints_x,map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60,(2+4*lane),map_waypoints_s, map_waypoints_x,map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90,(2+4*lane),map_waypoints_s, map_waypoints_x,map_waypoints_y);
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
            
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          
          for (int i=0; i<ptsx.size(); i++){
            //shift car reference angle to 0 degree
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;
            
            ptsx[i]=(shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
            ptsy[i]=(shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
          } // M: Q: why we should creat?
          
          // creat a spline
          tk::spline s;
          
          s.set_points(ptsx, ptsy);
          
//           vector<double> next_x_vals;
//           vector<double> next_y_vals;
          
          for (int i=0; i<previous_path_x.size(); i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          // calculate how to break up spline points so that we travel at our desired reference velocity
          double target_x = 30; //M:Q: why only 30, maybe we would like to plan 30m ahead
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x+target_y*target_y);
          
          double x_add_on = 0;
          
          // Fill up the rest of path planner
          
          for (int i=1; i<=50-previous_path_x.size(); i++){
            double N = target_dist/(0.02*ref_vel/2.24);
            double x_point = x_add_on+(target_x)/N;
            double y_point = s(x_point);
            x_add_on = x_point;
            
            double x_ref = x_point;
            double y_ref = y_point;
            
            // rotate back to normal after rotating it earlier
            
            x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));
            
            x_point += ref_x;
            y_point += ref_y;
            
            next_x_vals.push_back(x_point); 
            next_y_vals.push_back(y_point); 
          }


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