#include "velocityProvider.hpp"
#include <cstdlib>

OtherClass::OtherClass()
: linear_x_(0.0f), angular_z_(0.0f)
{}

float OtherClass::getVel() {
	// update internal state here
	linear_x_ = 0.1f;
	return linear_x_;
}

float OtherClass::getRot(){
	angular_z_ = 0.0f;
	return angular_z_;
}
        // Make random numner between a and b
float rand_FloatRange(float a, float b)
    {
        return ((b - a) * ((float)rand() / RAND_MAX)) + a;
    }

