#ifndef ROAD_H
#define ROAD_H

#include "State.h"
#include <unordered_map>
#include "Vehicle.h"

class Road {
private:
	std::unordered_map<int, State> vehicles_;
public:
	Road();
	Road(const std::unordered_map<int, State> vehicles);

	std::unordered_map<int, State> getVehicles() const;

	void add(const Vehicle v);
	void remove(const Vehicle v);
};

#endif