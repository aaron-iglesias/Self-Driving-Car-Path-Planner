#include <vector>
#include "Vehicle.h"

class PathPlanner {
public:
	std::vector<std::vector<double> > planPath(Vehicle ego, std::vector<Vehicle> vehicles);
};