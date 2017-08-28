#include "Road.h"

// TODO: assert waypoints.size() are all same
Road::Road(double max_s, double speed_limit, vector<double> waypoints_x, vector<double> waypoints_y, vector<double> waypoints_s, vector<double> waypoints_dx, vector<double> waypoints_dy) {
	max_s_ = max_s;
	speed_limit_ = speed_limit;
	waypoints_["x"] = waypoints_x;
	waypoints_["y"] = waypoints_y;
	waypoints_["s"] = waypoints_s;
	waypoints_["dx"] = waypoints_dx;
	waypoints_["dy"] = waypoints_dy;
}

double Road::getMaxS() const {
	return max_s_;
}

unordered_map< string, vector<double> > Road::getWaypoints() const {
	return waypoints_;
}