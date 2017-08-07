#ifndef STATE_H
#define STATE_H

struct State {
	double x;
	double y;
	double s;
	double d;
	double yaw;
	double speed;
	State(double x = 0, double y = 0, double s = 0, double d = 0, double yaw = 0, double speed = 0);
};

#endif