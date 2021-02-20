#include "vehicle.h"
#include <algorithm>
#include <iterator>
#include <map>
#include <string>
#include <vector>


using std::string;
using std::vector;

// Initializes Vehicle
Vehicle::Vehicle(){
  this->updated = 0;
}

Vehicle::Vehicle(int lane, float s, float v, float a, string state) {
  this->lane = lane;
  this->s = s;
  this->v = v;
  this->a = a;
  this->state = state;
  max_acceleration = -1;
  this->updated = 0;
}

Vehicle::~Vehicle() {}

