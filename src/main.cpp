#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <map>
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
//11111111111111111111111111111111111111111111111111111111111
float dt = 0; // keeps track of time between recieving data from simulator
int num_path_pts = 50; // number of points in path sent to simulator
int lane = 1; // car projected lane
double max_vel = (49.5)*(0.447); //(MPH)*(0.447 mph to m/s) max velocity of vechicle
double max_acc = 10; //m/s/s
double ref_vel = 0; //mph - used to set velocity of vehicle
double target_vel = max_vel; //mph - target velocity of vehicle - used when following another vehicle
double ch_vel = (0.5)*(0.447); //(MPH)*(0.447 mph to m/s) change in velocity per timestep speeding up or slowing down
std::map<int, Vehicle> vehicles; 
Vehicle ego; 
int ego_id = -1;
// The max s value before wrapping around the track back to 0
double max_s = 6945.554; // meters
//11111111111111111111111111111111111111111111111111111111111
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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
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

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

//222222222222222222222222222222222222222222222222222222222

          // sensor_fusion [ id, x, y, vx, vy, s, d]
          // Vehicle(int lane, float s, float v, float a, string state="CS");

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
          int prev_size = previous_path_x.size();    

          for(int i = 0; i < sensor_fusion.size(); i++){ 
            int v_id = sensor_fusion[i][0];
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double v = ConvertSpeed(vx,vy);
            double s = sensor_fusion[i][5];
            double d = sensor_fusion[i][6];
            int l = DetermineLane(d);
            if(vehicles.count(v_id)){ 
              vehicles[v_id].updated = 1;
              vehicles[v_id].lane = l;
              if(s < vehicles[v_id].s){
                vehicles[v_id].laps += 1;
              }
              vehicles[v_id].s = s + max_s*vehicles[v_id].laps;
              vehicles[v_id].a = CalclateAcceleration(v, vehicles[v_id].v, dt);
              vehicles[v_id].v = v;    
              vehicles[v_id].distance_to_ego = s - car_s;          
            } else {      
              if (v > 0) {
              Vehicle vehicle;
              vehicle.updated = 1;
              vehicle.lane = l;
              vehicle.s = s;
              vehicle.a = 0;
              vehicle.v = v;  
              vehicle.laps = 0;  
              vehicle.distance_to_ego = s - car_s;          
              vehicles.insert(std::pair<int,Vehicle>(v_id,vehicle));       
              }            
            }
          }// end add or update sensed vehicles 
          // start delete all vehicles not present in current update
          vector<int> not_updated;
          for(auto v: vehicles){
            //std::cout<<"vehicle: " << v.first << std::endl;
            if(v.second.updated == 0){
              not_updated.push_back(v.first);
            }
          }
          for(auto nu: not_updated){
            vehicles.erase(nu);
          } // end delete all vehicles not present in current update
// ........................................................ 

          int car_lane = DetermineLane(car_d);

          vector<std::map<int, float>>  close_in_lanes(3);
          for(auto v: vehicles){
            for (int i=0; i < 3; i++){
              int check_dist = 45;
              if (i == car_lane){
                check_dist = 30;
              }
              if((v.second.distance_to_ego > 0) && (v.second.distance_to_ego < check_dist)){
                close_in_lanes[v.second.lane].insert(std::pair<int, float>(v.first, v.second.distance_to_ego));
              }
            }            
          }
          vector<int> closest_in_lanes(3, -1);
          for(int i = 0; i < 3; i++){
            if(close_in_lanes[i].size()){ 
              int closest_dist = pow(2,16);
              for(auto v: close_in_lanes[i]){
                if(v.second < closest_dist){
                  closest_in_lanes[i] = v.first;
                  closest_dist = v.second;
                }
              }
            }
          }

          int cl_id = closest_in_lanes[car_lane];
          if(cl_id > -1){ // car is in front of ego - check if should change lane
             switch(car_lane){ // todo - consolidate repeating code
              case 0:
                if(closest_in_lanes[1] < 0){ // no car in middle lane - change lane to middle and target max velocity
                  lane = 1; 
                  target_vel = max_vel;                                   
                } else {  // car also in middle lane - follow fastest
                  if( vehicles[closest_in_lanes[1]].v > vehicles[closest_in_lanes[0]].v){
                    lane = 1;
                    target_vel = vehicles[closest_in_lanes[1]].v;
                  } else{
                    lane = 0;
                    target_vel = vehicles[closest_in_lanes[0]].v;
                  }
                }
                break;

              case 1:
                if(closest_in_lanes[0] < 0){ // no car in left lane - change lane to left and target max velocity
                lane = 0;
                target_vel = max_vel; 
                } else if (closest_in_lanes[2] < 0){ // no car in right lane - change lane to right and target max velocity
                lane = 2;
                target_vel = max_vel;                  
                } else { // cars in all lanes - follow fastest
                  int fastest_lane = 0;
                  float fastest_velocity = 0;
                  for(int i = 0; i < 3; i++){
                    if(vehicles[closest_in_lanes[i]].v > fastest_velocity){
                      fastest_velocity = vehicles[closest_in_lanes[i]].v;
                      fastest_lane = i;
                    }
                  }
                  lane = fastest_lane;
                  target_vel = vehicles[closest_in_lanes[fastest_lane]].v;
                }
                break;

              case 2:
                if(closest_in_lanes[1] < 0){ // no car in middle lane - change lane to middle and target max velocity
                  lane = 1; 
                  target_vel = max_vel;                                   
                } else {  // car also in middle lane - follow fastest
                  if( vehicles[closest_in_lanes[1]].v > vehicles[closest_in_lanes[2]].v){
                    lane = 1;
                    target_vel = vehicles[closest_in_lanes[1]].v;
                  } else{
                    lane = 2;
                    target_vel = vehicles[closest_in_lanes[2]].v;
                  }
                }
                break;
            }
          } else { // car is not in front of ego - target max velocity
            target_vel = max_vel;
          }
// ........................................................   
/* ////////////////////////////////////////////////
          vector<float> same_lane_dist(sensor_fusion.size(), 9999999.9);
          int found_close = 0;
          for(int i = 0; i < sensor_fusion.size(); i++){ // start go through sensed vehicles
            float d = sensor_fusion[i][6];
            if (lane == DetermineLane(d)){
              double check_car_s = sensor_fusion[i][5];
              if((check_car_s > car_s)&&((check_car_s-car_s)<30)){
                same_lane_dist[i] = check_car_s-car_s;
                found_close = 1;  
              }
            }
          } // end go through sensed vehicles        
          
          if(found_close){
              std::vector<float>::iterator closest = std::min_element(same_lane_dist.begin(), same_lane_dist.end());
              int closest_id = std::distance(same_lane_dist.begin(), closest);
              //std::cout << "closest id : " << closest_id << " closest_s: " << sensor_fusion[closest_id][5] << " car_s: " <<  car_s << std::endl;
              double vx = sensor_fusion[closest_id][3];
              double vy = sensor_fusion[closest_id][4];
              double check_speed = ConvertSpeed(vx,vy);
              if(check_speed < max_vel ){
                target_vel = check_speed;
              } else {
                target_vel = max_vel;
              }
          } else {
            target_vel = max_vel;
          }
*/////////////////////////////////////////////////
          if(target_vel > ref_vel + 0.5*max_acc*0.02){
            ref_vel = ref_vel + max_acc*0.02;
          } else if (target_vel < ref_vel - 0.5*max_acc*0.02) {
            ref_vel = ref_vel - max_acc*0.02;
          } else {
            ref_vel = target_vel;
          }

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// ------------------------------------------------------------

          if(prev_size > 0){
            car_s = end_path_s;
          }
          // wideley spaced waypoints for spline to interpolate
          vector<double> ptsx;
          vector<double> ptsy;
          // reference x,y, yaw states
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
 
          // if previous does not have enough points to use then use the car starting reference
          if(prev_size < 2){ // use the car position to start the vector
            //use two point that make the path tangent to the car
            
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          } else {  // use last previous points to start the vector
            //redefine refernce state as previou path end point
            ref_x = previous_path_x[prev_size -1];
            ref_y = previous_path_y[prev_size -1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }
          int lane_center_d = 2 + 4*lane;
          vector<double> next_wp0 = getXY(car_s+30,lane_center_d,map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60,lane_center_d,map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90,lane_center_d,map_waypoints_s,map_waypoints_x,map_waypoints_y);
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          for (int i = 0; i < ptsx.size(); i++){

            //shift car angle reference to 0 degrees
            //offset to reference
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;
            //apply rotational matrix with negative yaw
            ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
          }

          // create spline
          tk::spline s;
          // set x,y points to the spline
          s.set_points(ptsx,ptsy);

          //start with all the previous path points that where not used
          for(int i = 0; i < prev_size; i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          // break up spline points according to velocity
          double target_x = 30.0;
          double target_y=s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);

          double x_add_on = 0;

          // complete the path planner with spline points

          for (int i =1; i <= num_path_pts-prev_size; i++){
  
            double N = target_dist/(0.02*ref_vel); 
            double x_point = x_add_on+target_x/N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // rotate back to normal after rotating earlier
            x_point = x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw);
            y_point = x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw);

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
          //---------------------------------------------------------------------------
          for (auto v: vehicles){ 
            v.second.updated = 0;
          }          
          //222222222222222222222222222222222222222222222222222222222
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