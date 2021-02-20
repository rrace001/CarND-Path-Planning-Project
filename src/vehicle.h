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
  Vehicle(int lane, float s, float v, float a, string state="CS");

  // Destructor
  virtual ~Vehicle();

  // Vehicle functions
  
  int L = 1;

  int preferred_buffer = 45; // impacts "keep lane" behavior. meters

  int lane, goal_lane = 1, lanes_available = 3, updated = 0, laps;

  float goal_s, s, v, target_speed, a, max_acceleration, distance_to_ego;

  string state;
};

#endif  // VEHICLE_H