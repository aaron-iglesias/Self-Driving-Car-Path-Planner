#include "Vehicle.h"

Vehicle::Vehicle(const unsigned int v_id) : id(v_id) {}

Vehicle::Vehicle(const unsigned int v_id, const State state) : id(v_id) {
	state_ = state;
}

State Vehicle::getState() const {
	return state_;
}