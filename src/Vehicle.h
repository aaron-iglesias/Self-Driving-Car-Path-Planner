#ifndef VEHICLE_H
#define VEHICLE_H

#include "State.h"

class Vehicle {
private:
	State state_;
public:
	const unsigned int id;
	Vehicle(const unsigned int v_id);
	Vehicle(const unsigned int v_id, const State state);
	State getState() const;
};

#endif