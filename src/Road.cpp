#include "Road.h"

Road::Road() {}

Road::Road(const std::unordered_map<int, State> vehicles) {
	vehicles_ = vehicles;
}

std::unordered_map<int, State> Road::getVehicles() const {
	return vehicles_;
}

void Road::add(const Vehicle v) {
	vehicles_[v.id] = v.getState();
}

void Road::remove(const Vehicle v) {
	auto it = vehicles_.begin();
	if(it != vehicles_.end())
		vehicles_.erase(it);
}