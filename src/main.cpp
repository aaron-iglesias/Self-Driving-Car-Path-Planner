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
const unsigned int num_waypoints = 50;
const double dist_inc = 0.3;
const int lane = 1;
const double speed_limit = 50;

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

double distance(const double &x1, const double &y1, const double &x2, const double &y2) {
  return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
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

double min(const double &a, const double &b) {
  return a < b ? a : b;
}

// converts Lane to d Frenet coordinate
int LaneToD(const char &lane) {
  if(lane == 'l')
    return 2;
  if(lane == 'm')
    return 6;
  if(lane == 'r')
    return 10;
  return -1;
}

// uses sensor fusion data to find the closest front and back vehicles for current lane
vector<int> closestVehicles(const vector< vector<double> > &sensor_fusion, const double &car_x, const double &car_y, const double &car_yaw, const double &car_d) {
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

  // define important variables and constants
  vector<int> id_closest = {-1, -1};
  const int lane_num = car_d / 4;
  const vector<int> d_bound = {4 * lane_num, 4 * lane_num + 4};
  vector<double> dist_closest = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};

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
  double yaw_end = 1;

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

  double ref_vel = 0;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&yaw_end,&max_s,&ref_vel](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          
          // Main car's localization Data
            double car_x = j[1]["x"];
            double car_y = j[1]["y"];
            double car_s = j[1]["s"];
            double car_d = j[1]["d"];
            double car_yaw = j[1]["yaw"];
            double car_speed = j[1]["speed"];

            print(car_s);

            // Previous path data given to the Planner
            vector<double> previous_path_x = j[1]["previous_path_x"];
            vector<double> previous_path_y = j[1]["previous_path_y"];
            // Previous path's end s and d values 
            double end_path_s = j[1]["end_path_s"];
            double end_path_d = j[1]["end_path_d"];

            // Sensor Fusion Data, a list of all other cars on the same side of the road.
            vector< vector<double> >  sensor_fusion = j[1]["sensor_fusion"];

            json msgJson;

            // find closest car in front in same lane
            vector<int> id_closest = closestVehicles(sensor_fusion, car_x, car_y, degToRad(car_yaw), car_d);
            int id_closest_front = id_closest[1];
            print(id_closest[0]);
            print(id_closest_front);
            print();

            vector<double> ptsx, ptsy;

            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = degToRad(car_yaw);

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
            double x_add_on = 0;
            for(int i = prev_size, j = 0; i < num_waypoints; ++i) {
              bool going_slower = false;
              bool too_close = false;
              if(id_closest_front != -1) {
                going_slower = sqrt(pow(sensor_fusion[id_closest_front][3], 2) + pow(sensor_fusion[id_closest_front][4], 2)) < ref_vel;
                too_close = distance(next_x_vals.back(), next_y_vals.back(), sensor_fusion[id_closest_front][1], sensor_fusion[id_closest_front][2]) < 10;
              }
              bool slow_down = going_slower && too_close;

              ref_vel = slow_down ? ref_vel - 0.4 : min(ref_vel + 0.4, speed_limit - 0.3);
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

            // convert local to global coordinates
            // vector< vector<double> > pts_global = localToGlobal(next_x_vals, next_y_vals, ref_x, ref_y, ref_yaw);
            // next_x_vals = pts_global[0];
            // next_y_vals = pts_global[1];

            // define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
            msgJson["next_x"] = next_x_vals;
            msgJson["next_y"] = next_y_vals;

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