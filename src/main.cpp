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

#define MIN(x, y) (((x) < (y)) ? (x) : (y))

const int LANE_WIDTH   = 4;    // 4 meters
const double SAFE_DIST = 30.0; // 30 meters
const double MAX_SPEED = 49.5; // 49.5 MPH
const int NUM_TRJ_PTS  = 50;   // 50 points

// Costs
const double COLLISION_COST = 10000;
const double SLOW_LANE_COST = 50;
const double TARGET_LANE_COST = 10;
const double TRAJECTORY_JERK_COST = 5000;

const double LANE_CHNG_BACK_CLEARNCE = 15;
const double LANE_CHNG_FRONT_CLEARNCE = 30;
const double VEHICLE_DISTANCE_FOR_LANE_SPEED = 100; // 100m

enum Lane
{
  LEFT     = 0,
  CENTER   = 1,
  RIGHT    = 2,
  INVALID  = 3,
};

enum class State {
  KL    = 0, // Keep Lane
  LCL   = 1, // Lane Change Left
  LCR   = 2, // Lane Change Right
};

struct Vehicle {
  int id;
  double s;   // Current position
  double d;
  double x;
  double y;
  double vx;
  double vy;
  
  Lane    lane;  
  double speed; // Speed along 's'
  double end_path_s; // Poition 's' at the end point
  
};

struct MyCar {
  double s;   // Current position
  double d;
  double x;
  double y;
  Lane   lane; 
  State  state;
  double end_path_s; 
  double speed;
};

// Returns the lane of the vehicle given its 'd'
Lane GetLane(double d) {
  if (d > 0 && d < LANE_WIDTH) {
    return Lane::LEFT;
  } else if (d > LANE_WIDTH && d < LANE_WIDTH*2) {
    return Lane::CENTER;
  } else if (d > LANE_WIDTH*2 && d < LANE_WIDTH*3) {
    return Lane::RIGHT;
  } else {
    return Lane::INVALID;
  }
}

// Given the list of other cars on the road, find the speed of 
// each lanes. Speed of the vehicle just ahead of us in that 
// lane is considered as the lane speed here.
vector<double> GetLaneSpeeds(double my_car_end_path_s, vector<Vehicle> &other_cars) {
  
  vector<double> lane_speeds         = {99999, 99999, 99999};
  vector<double> distance_to_nearest = {99999, 99999, 99999};
  
  for(Vehicle other_car : other_cars) {
    // Check if the vehicle is ahead of us and is within a disctance that we care about.
    if((other_car.end_path_s > my_car_end_path_s) && ((other_car.end_path_s - my_car_end_path_s) < VEHICLE_DISTANCE_FOR_LANE_SPEED)) {
      // Check if the other car is in valid lane - Just in case
      // we don't want to index into invalid location in our vector
      if(other_car.lane == Lane::INVALID) continue; // Skip this car
      
      double dist = other_car.end_path_s - my_car_end_path_s;
      if (dist < distance_to_nearest[other_car.lane]) {
        distance_to_nearest[other_car.lane] = dist;
        lane_speeds[other_car.lane]         = other_car.speed;
      }
    }
  }
  
  return lane_speeds;
}

// Given my car's lane and the next state, this function returns the target lane
Lane GetTargetLane(Lane my_car_lane, State next_state) {
  // Find the target lane
  Lane target_lane = my_car_lane;
  if(next_state == State::LCL)
    target_lane = (Lane)(my_car_lane - 1);
  else if (next_state == State::LCR)
    target_lane = (Lane)(my_car_lane + 1);
  
  return target_lane;  
}

// Returns the list of possible next states, 
// for the given lane and state
vector<State> PossibleNextStates(Lane l, State s) {
  vector<State> next_states;
  next_states.push_back(s); // Staying in the current state is valid for all states
  
  if(s == State::KL) {
    if(l == Lane::LEFT)
      next_states.push_back(State::LCR);
    else if (l == Lane::RIGHT)
      next_states.push_back(State::LCL);
    else { 
      // All the states are possible when we are in center lane and the current state is KL
      next_states.push_back(State::LCL);
      next_states.push_back(State::LCR);
    }
  } else {
    // If we are either in LCL or LCR state, only other 
    // allowed state other than itself is KL
    next_states.push_back(State::KL);
  }
  
  return next_states;
}

// This is a binary cost function.
// Outputs a very high cost if there are other cars in the target lane closeby.
double ComputeLaneChangeCollisionCost(MyCar my_car, vector<Vehicle> &other_cars, State next_state) {
  double cost = 0.0;
  
  // This cost is applicable only for lane changing states
  if(next_state == State::KL) return cost;
  
  // Find the target lane
  Lane target_lane = GetTargetLane(my_car.lane, next_state);
  
  for(Vehicle other_car : other_cars) {    
    if(other_car.lane == target_lane) {
      // Check if the other car is close by
      double lane_chng_back_clearance = LANE_CHNG_BACK_CLEARNCE;
      if((other_car.end_path_s > my_car.s) &&   // This car is behind us
         (other_car.speed > my_car.speed) &&    // This car is faster than us by more than 5MPH
         ((other_car.speed - my_car.speed) > 5)) {
        lane_chng_back_clearance = LANE_CHNG_BACK_CLEARNCE * 2;
      }
         
      if((other_car.end_path_s > (my_car.s - lane_chng_back_clearance)) && (other_car.end_path_s < (my_car.s + LANE_CHNG_FRONT_CLEARNCE))) {
        return COLLISION_COST;
      }
        
    }
  }
  
  return cost;
}

// Penalize the lanes that have lower speed
double ComputeLaneSpeedCost(MyCar my_car, vector<Vehicle> &other_cars, State next_state) {
  double cost = 0.0;
  
  // Lane speed is the speed of the vehicle just ahead of us in that lane
  vector<double> lane_speeds = GetLaneSpeeds(my_car.s, other_cars);
  
  Lane target_lane = GetTargetLane(my_car.lane, next_state);
  if(lane_speeds[target_lane] < MAX_SPEED) {
    // TODO: Here the cost linearly increases as the lane speed decreases from MAX Speed
    // Should we use a logistic function here instead of a linear function ?
    cost = (MAX_SPEED - lane_speeds[target_lane]) * SLOW_LANE_COST;
  }
  
  return cost;
}

// We prefer to be in the center lane when possible
double ComputeTargetLanecost(MyCar my_car, vector<Vehicle> &other_cars, State next_state) {
  double cost = 0.0;
  Lane target_lane = GetTargetLane(my_car.lane, next_state);
  if(target_lane != Lane::CENTER)
    cost = TARGET_LANE_COST;
  return cost;
}
int num_points_current_lane = 0;
double ComputeTrajectoryJerkCost(MyCar my_car, vector<Vehicle> &other_cars, State next_state) {
  // Penalize quick lane changes. Once we do a lane change, allow it to be in that lane for 
  // a while before allowing another lane change.
  double cost = 0.0;
  Lane target_lane = GetTargetLane(my_car.lane, next_state);
  if((target_lane != my_car.lane) && (num_points_current_lane < 250)) // Stay in a lane for atleast 5 5ec (20msec * 250)
    cost = TRAJECTORY_JERK_COST;
  
  return cost;
}

// Compute the overall cost for the given next state
double ComputeCost(MyCar my_car, vector<Vehicle> &other_cars, State next_state) {
  double cost = 0.0;  
  double total_cost = 0.0;
  cost = ComputeLaneChangeCollisionCost(my_car, other_cars, next_state);
  //std::cout << "Costs: Collision=" << cost;
  total_cost += cost;
  cost = ComputeLaneSpeedCost(my_car, other_cars, next_state);
  //std::cout << " LaneSpeed=" << cost;
  total_cost += cost;
  cost = ComputeTargetLanecost(my_car, other_cars, next_state);
  //std::cout << " TargetLane=" << cost << std::endl;
  total_cost += cost;
  cost = ComputeTrajectoryJerkCost(my_car, other_cars, next_state);
  total_cost += cost;
  return total_cost;
}

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

  State state = State::KL; // Start with KL 
  int lane = Lane::CENTER; // Start in middle lane
  double ref_vel = 0.0; // Set the iniitial velocity to 0
    
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel, &state]
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
          double end_path_s = j[1]["end_path_s"] ;
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road. 
          // The data format for each car is: [ id, x, y, vx, vy, s, d]
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          // Fill this vector with the (x,y) points along the trajectory
          // that our ego car will traverse inside the simulator.
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          int prev_size = previous_path_x.size();
          if (prev_size > 0) {
            car_s = end_path_s;
          }
          
          // DEBUG
          /*
          std::cout << "Previous points:" << std::endl;
          for (int i = 0; i < prev_size; ++i)
            std::cout << "(" << previous_path_x[i] << "," << previous_path_y[i] << ")";
          std::cout << std::endl;
          */
          
          //////////////////////////////////////////////////////////////////////////////
          // Prediction:
          // Create the Vehicle objects, compute their future position based on velocity
          //////////////////////////////////////////////////////////////////////////////
          vector<Vehicle> other_cars;
          for (int i = 0; i < sensor_fusion.size(); i++) {
            Vehicle other_car;
            other_car.id = sensor_fusion[i][0];
            other_car.x  = sensor_fusion[i][1];
            other_car.y  = sensor_fusion[i][2];
            other_car.vx = sensor_fusion[i][3];
            other_car.vy = sensor_fusion[i][4];
            other_car.s  = sensor_fusion[i][5];
            other_car.d  = sensor_fusion[i][6];
            
            // It is convenient to save which lane this car is in. It helps to avoid
            // repeated calculations in future. I am making assumption that the other
            // vehicles are not doing a lane change and stay in its lane
            other_car.lane = GetLane(other_car.d);
            
            // Compute the speed of this car
            other_car.speed = sqrt(other_car.vx*other_car.vx + other_car.vy*other_car.vy); 
            
            // If there are previous points available, project this cars 
            // position after the previous points are consumed.
            // Making a very simple prediction assuming constant velocity.
            other_car.end_path_s = other_car.s + ((double)prev_size) * 0.02 * other_car.speed;
            
            other_cars.push_back(other_car);            
          }
          
          // Create my_car object
          MyCar my_car;
          my_car.x = car_x;
          my_car.y = car_y;
          my_car.s = car_s;
          my_car.d = car_d;
          my_car.lane = (Lane)lane;
          my_car.state = state;
          my_car.end_path_s = car_s; 
          my_car.speed = ref_vel;
          
          
          //////////////////////////////////////////////////////////////////////////////
          // Behaviour Planning
          //  - Finite State machine
          //  - Cost functions
          //////////////////////////////////////////////////////////////////////////////
          
          // Get the possible next states
          vector<State> next_states = PossibleNextStates((Lane)lane, state);
          
          double lowest_cost = 99999;
          State  best_state  = State::KL;
          
          // Compute the cost of each of the next states and 
          // find the next state with minimum cost
          for (auto next_state: next_states){
            double cost = ComputeCost(my_car, other_cars, next_state);
            if(cost < lowest_cost) {
              lowest_cost = cost;
              best_state  = next_state;
            }
          }
          
          ////////////////////////////////////////////////////////////////////////////////////////       
          // Trajectory Generation
          //  - Based on the selected next state, compute smooth trajectory for our car
          ////////////////////////////////////////////////////////////////////////////////////////
          
          bool too_close = false;
          double target_speed = MAX_SPEED;
          if(best_state == State::LCL) { lane -= 1; num_points_current_lane = 0; }
          if(best_state == State::LCR) { lane += 1; num_points_current_lane = 0; }
          if(best_state == State::KL) {
            // if we are following a car in front of us, 
            // try to slow down and match with that car's speed
            for(auto other_car : other_cars) {
              /*
              if(other_car.lane == lane) {
                std::cout << "Found car in lane. Other car end s = " << other_car.s << " My car s = " << my_car.s << std::endl;
              }
              */
              if((other_car.lane == lane) && (other_car.end_path_s > my_car.s) && ((other_car.end_path_s - my_car.s) < SAFE_DIST)) {
                // We are following a car
                if (other_car.speed < target_speed)
                  target_speed = other_car.speed;
                if((other_car.end_path_s - my_car.s) < 15) // too close - brake harder ?
                  too_close = true;
              }
            }
          }
          
          std::cout << "Best state = " << (int)best_state <<  " Lane = " << lane << " target_speed = " << target_speed << std::endl;
          
          // Points to generate trajectory using spline interpolation library
          vector<double> ptsx;
          vector<double> ptsy;
          
          // Refernce x, y, yaw states
          double ref_x, ref_y, ref_yaw;
          
          // Add last two points from the previous points if available.
          if (previous_path_x.size() >= 2)
          {
            // Lets use the last point in the previous points as the reference
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];
              
            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
            
            // Lets use last two points from previous iteration to make the trajectory continous
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }
          else
          {
            // Use the curent car position as the reference
            ref_x = car_x;
            ref_y = car_y;
            ref_yaw = deg2rad(car_yaw);
            
            // Estimate one previous point based on the current car position and the 
            // current car orientation.
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            
            // Use these two points to fit the spline to make it continous
            ptsx.push_back(prev_car_x);
            ptsx.push_back(ref_x);
            
            ptsy.push_back(prev_car_y);
            ptsy.push_back(ref_y);
          }
          
          
          // Add three more waypoints spaced 30m ahead of us. It is easier to find
          // these points in frenet cordinate first and then use the getXY() to convert 
          // to global (x,y) coordinates before adding it to ptsx,ptsy vectors.
          // Also note that the 'd' is calculated based on the future lane decided by behaviour layer.
          vector<double> next_wp0 = getXY(car_s+30, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          // Add these (x,y) points to the ptsx,ptsy for spline
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          
          // convert the points to cars local coordinates.
          // In local coordinates, car's current position is 0,0 and 
          // orientation is 0 degree.
          // This is to simplify the math. We will have to convert it back to 
          // global coordinates before sending these values to simulator
          for (int i = 0; i < ptsx.size(); i ++)
          {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            
            ptsx[i] = shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw);
            ptsy[i] = shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw);
          }
          
          // DEBUG: Print the points
          /*
          std::cout << "Points to spline:" << std::endl;
          for (int i = 0; i < ptsx.size(); i++)
            std::cout << "(" << ptsx[i] << "," << ptsy[i] << ")";
          std::cout << std::endl;
          */
          
          tk::spline s;             // create the spline object
          s.set_points(ptsx, ptsy); // set the points to fit
          
          // Add all the points left over from previous iteration
          for (int i = 0; i < prev_size; ++i)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          // Calculate how to break up the spline so that we travel at the desired velocity and not exceeding
          // the acceleration and jerk limits
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_distance = sqrt(target_x*target_x + target_y*target_y); // hypotenuse 
          
          double x_add_on = 0; // Start at 0, since we are in local coordinates
          num_points_current_lane += NUM_TRJ_PTS-prev_size;
          // Fill up rest of the points complete the trajectory
          for (int i = 0; i < NUM_TRJ_PTS-prev_size; i++)
          {
            if(target_speed > ref_vel) {
              ref_vel = ref_vel + MIN(0.3, target_speed-ref_vel);
            } else if (target_speed < ref_vel) {
              ref_vel = ref_vel - MIN(0.3, ref_vel-target_speed);
            }
            
            if(ref_vel > MAX_SPEED) ref_vel = MAX_SPEED; // Make sure we are not over speeding
              
            double N = target_distance/(0.02*ref_vel/2.24);
            double x_point = x_add_on+(target_x/N);
            double y_point = s(x_point);
            
            // Update the x_add_on to current x_point
            x_add_on = x_point;
            
            // Convert the points back to global coordinates
            double x_ref = x_point;
            double y_ref = y_point;
            
            x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
            y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);
            
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