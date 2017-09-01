#include <fstream>
#include <string>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include <algorithm>
#include "print.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// define constants
const unsigned int d = 6;
const unsigned int num_spline_points = 5;
const unsigned int spline_point_dist = 30;
const unsigned int num_waypoints = 25;
const double dist_inc = 0.3;

const double speed_limit = 49.85;
const double speed_limit_buffer = 0.15;
// distance to keep between vehicles
const double safety_gap = 35;

const double lane_inc = 0.01;
const double count_inc = 100 * lane_inc;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double degToRad(double x) { return x * pi() / 180; }
double radToDeg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

bool isInt(const double &n) {
  return floor(n) == n;
}

double speed(const double &vx, const double &vy) {
  return sqrt(pow(vx, 2) + pow(vy, 2));
}

double speed(const vector< vector<double> > &sensor_fusion, const int &id) {
  const int vx = sensor_fusion[id][3];
  const int vy = sensor_fusion[id][4];
  return sqrt(pow(vx, 2) + pow(vy, 2));
}

double distance(const double &x1, const double &y1, const double &x2, const double &y2) {
  return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}


double sDistance(double a, double b, const double max_s = 6945.554) {
  double res = abs(a - b);
  if(res > max_s / 2) {
    if(a >= 0)
      a += max_s;
    else
      b += max_s;
    res = abs(a - b);
  }
  return res;
} 

int ClosestWaypoint(const double &x, const double &y, const vector<double> &maps_x, const vector<double> &maps_y)
{

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(int i = 0; i < maps_x.size(); i++)
  {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if(dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }

  }

  return closestWaypoint;

}

int NextWaypoint(const double &x, const double &y, const double &theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2( (map_y-y),(map_x-x) );

  double angle = abs(theta-heading);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  }

  return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(const double &x, const double &y, const double &theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0)
  {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if(centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++)
  {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(const double &s, const double &d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
  int prev_wp = -1;

  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
  {
    prev_wp++;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

// TODO: check x_global.size() == y_global.size()
vector< vector<double> > globalToLocal(const vector<double> &x_global, const vector<double> &y_global, const double &car_x, const double &car_y, const double &car_yaw) {
  vector<double> x_local(x_global.size());
  vector<double> y_local(y_global.size());

  for(int i = 0; i < x_local.size(); ++i) {
    double x_scale = x_global[i] - car_x;
    double y_scale = y_global[i] - car_y;
    double r = sqrt(pow(x_scale, 2) + pow(y_scale, 2));
    double theta = atan2(y_scale, x_scale) - car_yaw;

    x_local[i] = r * cos(theta);
    y_local[i] = r * sin(theta);
    // std::cout << x_local[i] << std::endl;
  }

  return {x_local, y_local};
}

// TODO: check x_local.size() == y_local.size()
vector< vector<double > > localToGlobal(const vector<double> &x_local, const vector<double> &y_local, const double &car_x, const double &car_y, const double &car_yaw) {
  vector<double> x_global(x_local.size());
  vector<double> y_global(y_local.size());

  for(int i = 0; i < x_global.size(); ++i) {
    double r = sqrt(pow(x_local[i], 2) + pow(y_local[i], 2));
    double theta = atan2(y_local[i], x_local[i]) + car_yaw;
    x_global[i] = r * cos(theta) + car_x;
    y_global[i] = r * sin(theta) + car_y;
  }

  return {x_global, y_global};
}

// converts mph to distance between points
double mphToDistInc(const double &mph) {
  return 8.94077777777777e-3 * mph;
}

double max(const double &a, const double &b) {
  return a > b ? a : b;
}

double max(const double &a, const double &b, const double &c) {
  double res = max(a, b);
  return max(res, c);
}

double min(const double &a, const double &b) {
  return a < b ? a : b;
}

int dToLane(const double d) {
  return d / 4;
}

// uses sensor fusion data to find the closest front and back vehicles for current lane
vector<int> findCloseVehiclesInLane(const vector< vector<double> > &sensor_fusion, const double &car_x, const double &car_y, const double &car_yaw, const double &car_d) {
  // get x, y points
  vector<double> x, y;
  for(const vector<double> &v : sensor_fusion) {
    x.push_back(v[1]);
    y.push_back(v[2]);
  }

  // convert x, y to local coordinates 
  vector< vector<double> > xy_local = globalToLocal(x, y, car_x, car_y, car_yaw);
  vector<double> x_local = xy_local[0];
  vector<double> y_local = xy_local[1];

  // define variables and constants
  vector<int> id_closest = {-1, -1};
  vector<double> dist_closest = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
  const int lane_num = car_d / 4;
  const vector<int> d_bound = {4 * lane_num, 4 * lane_num + 4};

  // find closest front and back vehicle in same lane
  for(int i = 0; i < sensor_fusion.size(); ++i) {
    bool is_same_lane = sensor_fusion[i][6] >= d_bound[0] && sensor_fusion[i][6] <= d_bound[1];
    if(!is_same_lane)
      continue;
    bool is_behind = x_local[i] < 0;
    double dist = distance(car_x, car_y, sensor_fusion[i][1], sensor_fusion[i][2]);
    if(is_behind && dist < dist_closest[0]) {
      dist_closest[0] = dist;
      id_closest[0] = sensor_fusion[i][0];
    }
    else if(!is_behind && dist < dist_closest[1]) {
      dist_closest[1] = dist;
      id_closest[1] = sensor_fusion[i][0];
    }
  }
  return id_closest;
}

vector< vector<int> > findCloseVehicles(const vector< vector<double> > &sensor_fusion, const double &car_x, const double &car_y, const double &car_yaw) {
  vector< vector<int> > res(3);
  for(int i = 0, d = 2; i < 3; ++i, d += 4) {
    const vector<int> frontAndBack = findCloseVehiclesInLane(sensor_fusion, car_x, car_y, car_yaw, d);
    res[i] = frontAndBack;
  }
  return res;
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

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
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

  double lane = 1;
  double ref_vel = 0;
  vector<bool> lane_change = {false, false};  
  double speed_limit_tmp = speed_limit;
  bool sl_flag = false;

  unsigned int cycle = 0;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&max_s,&ref_vel,&lane,&lane_change,&cycle,&speed_limit_tmp, &sl_flag](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          cycle = (cycle + 1) % 100;
          // Main car's localization Data
            double car_x = j[1]["x"];
            double car_y = j[1]["y"];
            double car_s = j[1]["s"];
            double car_d = j[1]["d"];
            double car_yaw = j[1]["yaw"];
            double car_yaw_rad = degToRad(car_yaw);
            double car_speed = j[1]["speed"];

            // Previous path data given to the Planner
            vector<double> previous_path_x = j[1]["previous_path_x"];
            vector<double> previous_path_y = j[1]["previous_path_y"];
            // Previous path's end s and d values 
            double end_path_s = j[1]["end_path_s"];
            double end_path_d = j[1]["end_path_d"];

            // Sensor Fusion Data, a list of all other cars on the same side of the road.
            vector< vector<double> >  sensor_fusion = j[1]["sensor_fusion"];

            json msgJson;

            // find closest vehicles to ego
            vector< vector<int> > close_vehicles = findCloseVehicles(sensor_fusion, car_x, car_y, car_yaw_rad);
            int front = close_vehicles[round(lane)][1];
            int front_left = -1, front_right = -1;
            int front_left_left = -1, front_right_right = -1;
            int back_left = -1, back_right = -1;
            int back_left_left = -1, back_right_right = -1;
            if(round(lane) != 0)
              front_left = close_vehicles[round(lane - 1)][1], back_left = close_vehicles[round(lane - 1)][0];
            else {
              front_right_right = close_vehicles[round(lane + 2)][1], back_right_right = close_vehicles[round(lane + 2)][0];
            }
            if(round(lane) != 2)
              front_right = close_vehicles[round(lane + 1)][1], back_right = close_vehicles[round(lane + 1)][0];
            else {
              front_left_left = close_vehicles[round(lane - 2)][1], back_left_left = close_vehicles[round(lane - 2)][0];
            }

            // ***** START *****

            // consider lane change
            if(abs(lane - round(lane)) < 0.01) {
              speed_limit_tmp = speed_limit;
              sl_flag = false;
              lane_change = {false, false};
              lane = round(lane);
              if(front != -1) {
                bool front_left_safe  = front_left == -1  || sDistance(car_s, sensor_fusion[front_left][5]) > safety_gap / 2  || (speed(sensor_fusion, front_left) > ref_vel  && sDistance(car_s, sensor_fusion[front_left][5]) > safety_gap / 4);
                bool front_right_safe = front_right == -1 || sDistance(car_s, sensor_fusion[front_right][5]) > safety_gap / 2 || (speed(sensor_fusion, front_right) > ref_vel && sDistance(car_s, sensor_fusion[front_right][5]) > safety_gap / 4);
                bool back_left_safe   = back_left == -1   || sDistance(car_s, sensor_fusion[back_left][5]) > safety_gap / 2   || (speed(sensor_fusion, back_left) < ref_vel   && sDistance(car_s, sensor_fusion[back_left][5]) > safety_gap / 6);
                bool back_right_safe  = back_right == -1  || sDistance(car_s, sensor_fusion[back_right][5]) > safety_gap / 2  || (speed(sensor_fusion, back_right) < ref_vel  && sDistance(car_s, sensor_fusion[back_right][5]) > safety_gap / 6);

                // if non-existent
                if(lane != 0 && front_left == -1 && back_left_safe)
                  lane_change[0] = true;
                else if(lane != 2 && front_right == -1 && back_right_safe)
                  lane_change[1] = true;
                if(!lane_change[0] && !lane_change[1]) {
                  // choose lane of furthest vehicle
                  double front_dist = sDistance(sensor_fusion[front][5], car_s);
                  double front_left_dist = front_left == -1 ? -1 : sDistance(sensor_fusion[front_left][5], car_s);
                  double front_right_dist = front_right == -1 ? -1 : sDistance(sensor_fusion[front_right][5], car_s);
                  double max_dist = max(front_dist, front_left_dist, front_right_dist);
                  int max_dist_id = front;
                  if(front_left_dist == max_dist)
                    max_dist_id = front_left;
                  else if(front_right_dist == max_dist)
                    max_dist_id = front_right;
                  if(lane == 1) {
                    if(max_dist_id == front_left && front_left_safe && back_left_safe)
                      lane_change[0] = true;
                    else if(max_dist_id == front_right && front_right_safe && back_right_safe)
                      lane_change[1] = true;
                  }
                  else if(lane == 0) {
                    double front_right_right_dist = front_right_right == -1 ? -1 : sDistance(sensor_fusion[front_right_right][5], car_s);
                    if(front_right_right_dist > max_dist) {
                      front_right_right_dist = max_dist;
                      max_dist_id = front_right_right;
                    }
                    if((max_dist_id == front_right || (max_dist_id == front_right_right)) && front_right_safe && back_right_safe)
                      lane_change[1] = true;
                  }
                  else if(lane == 2) {
                    double front_left_left_dist = front_left_left == -1 ? -1 : sDistance(sensor_fusion[front_left_left][5], car_s);
                    if(front_left_left_dist > max_dist) {
                      front_left_left_dist = max_dist;
                      max_dist_id = front_left_left;
                    }
                    if((max_dist_id == front_left || (max_dist_id == front_left_left)) && front_right_safe && back_left_safe)
                      lane_change[0] = true;
                  }
                }
              }
            }

            // update lane value
            if(lane_change[1]) 
              lane += lane_inc;
            else if(lane_change[0]) 
              lane -= lane_inc;
            if(abs(lane - round(lane)) < 0.01)
              lane = round(lane);

            // ***** END *****
            if(front != -1 && sDistance(sensor_fusion[front][5], car_s) < safety_gap / 2) {
              speed_limit_tmp = speed(sensor_fusion, front);
              sl_flag = true;
            }

            if(ref_vel > speed_limit_tmp) {
              ref_vel = max(ref_vel - 0.22, speed_limit_tmp);
            }
            else if(!sl_flag) {
              ref_vel = min(ref_vel + 0.4, speed_limit_tmp);
            }
            print(speed_limit_tmp);
            print(ref_vel);
            print();

            vector<double> ptsx, ptsy;

            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = car_yaw_rad;

            int prev_size = previous_path_x.size();
            if(prev_size < 2) {
              // use two points that make the path tangent to the car
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);

              ptsx.push_back(prev_car_x);
              ptsx.push_back(car_x);

              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);
            }
            // use the previous path's end point as starting reference
            else {
              // redefine reference state as previous path end point
              ref_x = previous_path_x[prev_size - 1];
              ref_y = previous_path_y[prev_size - 1];

              double ref_x_prev = previous_path_x[prev_size - 2];
              double ref_y_prev = previous_path_y[prev_size - 2];
              ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

              // use two points that make the path tangent to the previous path's end point
              ptsx.push_back(ref_x_prev);
              ptsx.push_back(ref_x);

              ptsy.push_back(ref_y_prev);
              ptsy.push_back(ref_y);
            }

            // create points for spline
            for(int i = ptsx.size(), j = 1; i < num_spline_points; ++i, ++j) {
              vector<double> next_wp = getXY(car_s + j * spline_point_dist, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              ptsx.push_back(next_wp[0]);
              ptsy.push_back(next_wp[1]);
            }

            // convert global to local coordinates
            vector< vector<double> > pts_local = globalToLocal(ptsx, ptsy, ref_x, ref_y, ref_yaw);
            ptsx = pts_local[0];
            ptsy = pts_local[1];

            // set points to spline
            tk::spline s;
            s.set_points(ptsx, ptsy);

            // add previous points to current points for planner
            vector<double> next_x_vals(previous_path_x);
            vector<double> next_y_vals(previous_path_y);

            // sample spline
            bool flag = false;
            double x_add_on = 0;
            for(int i = prev_size, j = 0; i < num_waypoints; ++i) {

              double x_point = x_add_on + mphToDistInc(ref_vel);
              double y_point = s(x_point);

              x_add_on = x_point;

              // center reference
              double x_ref = x_point;
              double y_ref = y_point;

              // // local to global coordinates
              x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
              y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }

            // define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
            msgJson["next_x"] = next_x_vals;
            msgJson["next_y"] = next_y_vals;
            // cycle += 1;

            auto msg = "42[\"control\","+ msgJson.dump()+"]";

            //this_thread::sleep_for(chrono::milliseconds(1000));
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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