#include "State.h"

class Vehicle {
private:
	State state_;
public:
	Vehicle();
	Vehicle(State state);
	State getState() const;
};