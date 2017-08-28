#ifndef ROAD_H
#define ROAD_H

#include <string>
#include <unordered_map>
#include <vector>

using namespace std;

class Road {
private:
	// TODO: make max_s_ constant
	double max_s_;
	double speed_limit_;
	unordered_map< string, vector<double> > waypoints_;
public:
	Road(double max_s, double speed_limit, vector<double> waypoints_x, vector<double> waypoints_y, vector<double> waypoints_s, vector<double> waypoints_dx, vector<double> waypoints_dy);

	double getMaxS() const;
	double getSpeedLimit() const;
	unordered_map< string, vector<double> > getWaypoints() const;
};

#endif