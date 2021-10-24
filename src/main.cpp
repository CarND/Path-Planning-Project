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
#include "vehicle.h"

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
	
  int lane = 1;
  double ref_vel = 0;
  
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel, &max_s]
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

          // speed limit
          double max_vel = 49.5;  
          // double delta_vel = .224; // change in velocity .224 ~ 5m/s^2 (limit 10)
          double a = 0; // acceleration of all cars initially 0
          
          double max_accel = 0.224;
          
          json msgJson;
          
          /**
           * @brief Determine current car (ego) lane.
           * 
           */

          // between 0 and 4  
          if (car_d < (2+4*0+2) && car_d > (2+4*0-2) ) 
          {
            lane = 0;
          }
          // between 4 and 8
          else if (car_d < (2+4*1+2) && car_d > (2+4*1-2) ) 
          {
            lane = 1;
          }
          // between 8 and 12
          else if (car_d < (2+4*2+2) && car_d > (2+4*2-2) ) 
          {
            lane = 2;
          }

          // instantiant Vehicle with lane, s, v, a, and Keep Lane 
          Vehicle ego = Vehicle(lane, car_s, car_speed, a, "KL");
          // set the goal of ego vehicle (needed for cost calculation)
          int safe_distance = 50;
          int passing_safe_distance = 50;
          ego.goal_s = max_s;
          ego.target_speed = max_vel;
          ego.max_acceleration = max_accel;
          ego.preferred_buffer = safe_distance;
          // ego.state = "KL";

          int prev_size = previous_path_x.size();

          if (prev_size > 0) 
          {
            car_s = end_path_s;
          }

          bool caution_ahead = false;  
          
          bool left_lane_open = true;
          bool center_lane_open = true;
          bool right_lane_open = true;
          double left_closest_s = 2*max_s;
          double center_closest_s = 2*max_s;
          double right_closest_s = 2*max_s;
          double left_closest_speed = max_vel*2;
          double center_closest_speed = max_vel*2;
          double right_closest_speed = max_vel*2;


          /**
           * @brief Get all other cars store in predictions.
           * 
           */
          vector<Vehicle> predictions;
          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          for (int i=0; i < sensor_fusion.size(); i++) 
          {
            int check_lane;
            float check_d = sensor_fusion[i][6];
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx+vy*vy);
            double check_car_s = sensor_fusion[i][5];
            
            // projected future s of other car
            check_car_s += ((double) prev_size*.02*check_speed); //if using previous points can project s value outwards in time

            /**
             * @brief Determine current car (ego) lane.
             * 
             */


            if (check_d < (2+4*0+2) && check_d > (2+4*0-2) ) // between 0 and 4
            {
              check_lane = 0;
            }
            
            else if (check_d < (2+4*1+2) && check_d > (2+4*1-2) ) // between 4 and 8
            {
              check_lane = 1;
            }
            
            else if (check_d < (2+4*2+2) && check_d > (2+4*2-2) ) // between 8 and 12
            {
              check_lane = 2;
            }

            // flag caution ahead if vehicle is in same lane as ego and closer than safe_zone
            if (check_lane == ego.lane && check_car_s > ego.s && check_car_s < ego.s + safe_distance && check_car_s > car_s)
            {
              caution_ahead = true;
            }    

            /**
             * @brief Checks if car is in center, left, or right lane, determines if it 
             * is open(safe to be in), and speed of near car ahead of ego for that lane
             * 
             */

            if (check_lane == 1) // center lane
            {
                // check if check vehicle is within +/- safe_distance of ego 
                // if (check_car_s > ego.s - safe_distance && check_car_s < ego.s + safe_distance)
                if (check_car_s >= ego.s && check_car_s < ego.s + passing_safe_distance)
                {
                  center_lane_open = false;

                  // update s and speed if closest to ego 
                  if (check_car_s > ego.s and check_car_s < center_closest_s)  
                  {
                    center_closest_s = check_car_s;
                    center_closest_speed = check_speed;
                  }
                  
                }    


            } 

            else if (check_lane == 0) // left lane  
            {
              // check if check vehicle is within +/- safe_distance of ego
              // if (check_car_s > ego.s - safe_distance && check_car_s < ego.s + safe_distance)
              if (check_car_s >= ego.s && check_car_s < ego.s + passing_safe_distance)
              {
                left_lane_open = false;

                // check if left car is ahead of ego and update speed if closest 
                if (check_car_s > ego.s and check_car_s < left_closest_s)  
                {
                  left_closest_s = check_car_s;
                  left_closest_speed = check_speed;
                }
              }    
            }
            else if (check_lane == 2) // right lane  
            {
              // check if check vehicle is within +/- safe_distance of ego
              // if (check_car_s > ego.s - safe_distance && check_car_s < ego.s + safe_distance)
              if (check_car_s >= ego.s && check_car_s < ego.s + passing_safe_distance)
              {
                right_lane_open = false;

                // check if right car is ahead of ego and update speed if closest 
                if (check_car_s > ego.s and check_car_s < right_closest_s)  
                {
                  right_closest_s = check_car_s;
                  right_closest_speed = check_speed;
                }

              }    

            } 


              double check_a = 0; // set acceleration of robot cars to 0

              // add robot car as Vehicle instance to predictions 
              predictions.push_back(Vehicle(check_lane, check_car_s, check_speed, check_a, "CS"));
          }

          // increase lane speed until max velocity reached if road ahead is clear
          if (caution_ahead == false) 
          {
            if(ref_vel < max_vel)
            {
              ref_vel += max_accel;
            }
          }
          // determine which open lane has highest speed and chose that lane
          else 
          {
            // determine which lanes are available lane change or keep lane
            // left_lane_open, center_lane_open, right_lane_open bool variables
            // and left_closest_speed, center_closest_speed, right_closest_speed variables
            // determined while looping through fusion data
            vector<bool> lanes_open { left_lane_open, center_lane_open, right_lane_open };

            // speed of vehicle directly ahead of ego in left, center, right lanes
            vector<double> lane_vehicle_speeds = { left_closest_speed, center_closest_speed, right_closest_speed};
            
            double vehicle_ahead_speed = lane_vehicle_speeds[lane];
            int fastest_lane = lane;

            // for each lane
            for (unsigned int i=0; i < lanes_open.size(); i++) 
            { 
              // and lane speed is faster than current lane speed
              // if (lane_vehicle_speeds[i] > vehicle_ahead_speed)
              if (lane_vehicle_speeds[i] > ego.v)
              {
                // move to new lane  
                fastest_lane = i;
              }

            }

            // fastest lane is 2 lanes across, move to middle lane if open
            if (fastest_lane - ego.lane == 2 or fastest_lane - ego.lane == -2) {
              if (center_lane_open) 
              {
                lane = 1;
              }
            }
            else 
            {
              // only move to target lane if open
              if (lanes_open[fastest_lane] == true)
              {
                // and if change in lanes at most 4m else jerk exceeded
                if ((4*fastest_lane-2) - car_d <= 4) {
                  lane = fastest_lane;
                }
              }    
            }


            if (ref_vel < lane_vehicle_speeds[lane] && ref_vel < lane_vehicle_speeds[ego.lane] && ref_vel < max_vel)
            {
              ref_vel += max_accel;
            } 
            else if (ref_vel > lane_vehicle_speeds[lane] && ref_vel > lane_vehicle_speeds[ego.lane])
            {
              if (ref_vel - lane_vehicle_speeds[lane] > max_accel)
              {
                // if speed difference too much, reduce by max acceleration
                ref_vel -= max_accel;
              }
              else {
                // reduce to speed of front car
                ref_vel = lane_vehicle_speeds[lane];   
              }

            }

            // std::cout << "next state lane "<< lane << std::endl;
            // std::cout << "next state reference velocity " << ref_vel << std::endl;            
          }


          // create list of widely spaced (x,y) waypoints evenly spaced at 30m
          // later we will interpolate these waypoints with a spline and fill it 
          // in with more points that control car
          vector<double> ptsx;
          vector<double> ptsy;

          // either we will reference starting point as where the car is or at 
          // the previous path or points
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // if previous path is almost empty, use car as starting reference
          if (prev_size < 2) {
            // use 2 points that make path tangent to car 
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);

          }
          // use previous path's end points as starting reference
          else
          {
            // redefine reference state as previous path end point
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_previous = previous_path_x[prev_size-2];
            double ref_y_previous = previous_path_y[prev_size-2]; 
            ref_yaw = atan2(ref_y-ref_y_previous, ref_x-ref_x_previous);

            ptsx.push_back(ref_x_previous);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_previous);
            ptsy.push_back(ref_y);
            std::cout << "NOT prev_size < 2: ref_x_previous "<< ref_x_previous << " ref_y_previous " << ref_y_previous << std::endl;
            std::cout << "NOT prev_size < 2: ref_x "<< ref_x << " ref_y " << ref_y << std::endl;
          }

          // in Frenet add evenly 30m spaced points ahead of the starting reference
          vector<double> next_wp0 = getXY(car_s+30,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          for (int i=0; i < ptsx.size(); i++) 
          {
            // shift car reference angles to 0 degrees
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw);
            ptsy[i] = shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw);            
            
            // std::cout << "shift_x pre "<< shift_x << "shift_y pre" << shift_y << std::endl;
            // std::cout << "ptsx[i]"<< ptsx[i] << "ptsy[i]" << ptsy[i] << std::endl;            
          }

          // create spline 
          tk::spline s;

          // set x,y points to the spline 
          s.set_points(ptsx, ptsy);

          // define the actual (x,y) points we will use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          for (int i=0; i < previous_path_x.size(); i++) 
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);            
          }

          // calculate how to break up spline points so that we travel at our 
          // desired reference velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x*target_x)+(target_y*target_y));

          double x_add_on = 0;

          // fill up the rest of our path planner after filling it with previous points
          // here we will always output 50 points
          for(int i = 0; i <= 50 - prev_size; ++i)
          {
            double N = target_dist / (0.02 * ref_vel/2.24);
            double x_point = x_add_on + target_x / N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // Rotating back to normal after rotating it earlier.
            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

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

