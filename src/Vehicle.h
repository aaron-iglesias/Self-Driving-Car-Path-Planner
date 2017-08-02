#include "State.h"

class Vehicle {
private:
	State state_;
public:
	State getState() const;
};