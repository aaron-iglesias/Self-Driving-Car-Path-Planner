#include "Vehicle.h"

Vehicle::Vehicle() {}

Vehicle::Vehicle(State state) {
	state_ = state;
}

State Vehicle::getState() const {
	return state_;
}