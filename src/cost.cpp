#include "cost.h"
#include <cmath>
#include <functional>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include "vehicle.h"
#include <stdio.h>
#include <iostream>

using std::string;
using std::vector;

/**
 * TODO: change weights for cost functions.
 */
const float REACH_GOAL = 0;
const float EFFICIENCY = 0;
const float SPEED = 1;  

float goal_distance_cost(const Vehicle &vehicle, 
                         const vector<Vehicle> &trajectory, 
                         const vector<Vehicle> &predictions, 
                         map<string, float> &data) {
  // Cost of being out of goal lane becomes larger as vehicle approaches 
  // goal distance.  There is no requirement for end lane so that cost 
  //  has been removed.  

  float cost;
  float distance_to_goal = data["distance_to_goal"];
  if (distance_to_goal > 0) {
    cost = 1 - 2*exp(-(2 / distance_to_goal));
  } else {
    cost = 1;
  }

  return cost;
}

float inefficiency_cost(const Vehicle &vehicle, 
                        const vector<Vehicle> &trajectory, 
                        const vector<Vehicle> &predictions, 
                        map<string, float> &data) {
  // Cost becomes higher for trajectories with intended lane and final lane 
  //   that have traffic slower than vehicle's target speed.
  // You can use the lane_speed function to determine the speed for a lane. 
  // This function is very similar to what you have already implemented in 
  //   the "Implement a Second Cost Function in C++" quiz.

  float proposed_speed_intended = lane_speed(vehicle, predictions, data["intended_lane"]);
  if (proposed_speed_intended < 0) {
    proposed_speed_intended = vehicle.target_speed;
  }

  float proposed_speed_final = lane_speed(vehicle, predictions, data["final_lane"]);
  if (proposed_speed_final < 0) {
    proposed_speed_final = vehicle.target_speed;
  }
    
  float cost = (2.0*vehicle.target_speed - proposed_speed_intended 
             - proposed_speed_final)/vehicle.target_speed;

  return cost;
}

float speed_cost(const Vehicle &vehicle, 
                        const vector<Vehicle> &trajectory, 
                        const vector<Vehicle> &predictions, 
                        map<string, float> &data) {
  // Cost becomes higher for trajectories with lower speed 
  float proposed_speed_intended = lane_speed(vehicle, predictions, data["intended_lane"]);
  if (proposed_speed_intended < 0) {
    proposed_speed_intended = vehicle.target_speed;
  }

  float proposed_speed_final = lane_speed(vehicle, predictions, data["final_lane"]);
  if (proposed_speed_final < 0) {
    proposed_speed_final = vehicle.target_speed;
  }

  // larger speed in final lane compared to intended lane drives cost down   
  double cost = 1 / (proposed_speed_final - proposed_speed_intended);

  return cost;
}

double lane_speed(const Vehicle &vehicle, const vector<Vehicle> &predictions, int lane) {
  
  double closest_car_speed = -1.0;
  double closest_s = vehicle.goal_s*2;

  // get speed of vehicle in front of and closest to ego in passed in lane 
  for (int i = 0; i < predictions.size(); ++i) {
    Vehicle other = predictions[i]; 
    if (other.lane == lane) {
      // check that vehicle ahead of ego and within safety buffer
      if (other.s > vehicle.s and other.s <= vehicle.s + vehicle.preferred_buffer)
      {
        if (other.s < closest_s)
        {
          closest_car_speed = other.v;
          closest_s = other.s;
        }    
      }
    }
  }
  // return closest car speed or -1.0 if no vehicles in lane and inside safety buffer
  return closest_car_speed;
}

float calculate_cost(const Vehicle &vehicle, 
                     const vector<Vehicle> &predictions, 
                     const vector<Vehicle> &trajectory) {
  // Sum weighted cost functions to get total cost for trajectory.
  map<string, float> trajectory_data = get_helper_data(vehicle, trajectory, 
                                                       predictions);
  
  float cost = 0.0;

  // cost function list - add additional cost functions here.
  vector<std::function<float(const Vehicle &, const vector<Vehicle> &, 
                             const vector<Vehicle> &, 
                             map<string, float> &)
    >> cf_list = {goal_distance_cost, inefficiency_cost, speed_cost};
  // weight list for multiplying cost function to prioritize 
  vector<float> weight_list = {REACH_GOAL, EFFICIENCY, SPEED};
    
  for (int i = 0; i < cf_list.size(); ++i) {
    float new_cost = weight_list[i]*cf_list[i](vehicle, trajectory, predictions, 
                                               trajectory_data);
    std::cout << "cost " << new_cost << " at i " << i << std::endl;                                          
    cost += new_cost;
  }

  return cost;
}

map<string, float> get_helper_data(const Vehicle &vehicle, 
                                   const vector<Vehicle> &trajectory, 
                                   const vector<Vehicle> &predictions) {
  // Generate helper data to use in cost functions:
  // intended_lane: the current lane +/- 1 if vehicle is planning or 
  //   executing a lane change.
  // final_lane: the lane of the vehicle at the end of the trajectory.
  // distance_to_goal: the distance of the vehicle to the goal.

  // Note that intended_lane and final_lane are both included to help 
  //   differentiate between planning and executing a lane change in the 
  //   cost functions.
  map<string, float> trajectory_data;
  Vehicle trajectory_last = trajectory[1];
  float intended_lane;

  if (trajectory_last.state.compare("PLCL") == 0) {
    intended_lane = trajectory_last.lane + 1;
  } else if (trajectory_last.state.compare("PLCR") == 0) {
    intended_lane = trajectory_last.lane - 1;
  } else {
    intended_lane = trajectory_last.lane;
  }

  float distance_to_goal = vehicle.goal_s - trajectory_last.s;
  float final_lane = trajectory_last.lane;
  trajectory_data["intended_lane"] = intended_lane;
  trajectory_data["final_lane"] = final_lane;
  trajectory_data["distance_to_goal"] = distance_to_goal;
    
  return trajectory_data;
}
