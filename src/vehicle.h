
#ifndef VEHICLE_H
#define VEHICLE_H

#include <map>
#include <string>
#include <vector>

using std::map;
using std::string;
using std::vector;

class Vehicle {
 public:
  // Constructors
  Vehicle();
  Vehicle(int lane, double s, double v, double a, string state="CS");

  // Destructor
  virtual ~Vehicle();

  // Vehicle functions
  vector<Vehicle> choose_next_state(vector<Vehicle> &predictions);

  vector<string> successor_states();

  vector<Vehicle> generate_trajectory(string state, 
                                      vector<Vehicle> &predictions);

  vector<double> get_kinematics(vector<Vehicle> &predictions, int lane);

  vector<Vehicle> constant_speed_trajectory();

  vector<Vehicle> keep_lane_trajectory(vector<Vehicle> &predictions);

  vector<Vehicle> lane_change_trajectory(string state, 
                                         vector<Vehicle> &predictions);

  vector<Vehicle> prep_lane_change_trajectory(string state, 
                                              vector<Vehicle> &predictions);

  void increment(int dt);

  double position_at(int t);

  bool get_vehicle_behind(vector<Vehicle> &predictions, int lane, 
                          Vehicle &rVehicle);

  bool get_vehicle_ahead(vector<Vehicle> &predictions, int lane, 
                         Vehicle &rVehicle);

 
  // public Vehicle variables
  struct collider{
    bool collision; // is there a collision?
    int  time; // time collision happens
  };

  map<string, int> lane_direction = {{"PLCL", 1}, {"LCL", 1}, 
                                     {"LCR", -1}, {"PLCR", -1}};

  int L = 1;

  int preferred_buffer = 6; // impacts "keep lane" behavior.

  int lane, goal_lane, lanes_available;

  double s, goal_s, v, target_speed, a, max_acceleration;

  string state;
};

#endif  // VEHICLE_H
