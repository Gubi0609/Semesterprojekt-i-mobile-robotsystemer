#include "velocityProvider.hpp"
#include <cstdlib>

VelocityProvider::VelocityProvider()
: linear_x_(0.0f), angular_z_(0.0f)
{}

float VelocityProvider::getVel() {
	return linear_x_;
}

float VelocityProvider::getRot(){
	return angular_z_;
}

void VelocityProvider::setVel(float f){
	linear_x_ = f;
}

void VelocityProvider::setRot(float f){
	angular_z_ = f;
}


