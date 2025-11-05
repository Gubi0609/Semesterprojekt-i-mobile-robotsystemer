#include "velocityProvider.hpp"
#include <cstdlib>

VelocityProvider::VelocityProvider()
: linear_x_(0.0f), angular_z_(0.0f), state_(State::IDLE), durationVelocity(0.0f)
{}

float VelocityProvider::getVel() {
	if(state_ == State::DURATION){
		if(std::chrono::steady_clock::now() <end_time_){
			//endtime is greater that timenow, which means not enough time has passed
			//endtime is evaluated at driveForDuration
			return durationVelocity;
		}
		else{ //once the duration has passed, the mode i set to idle
			state_ = State::IDLE;
			durationVelocity = 0.0f; //maybe setting linearx and returning it is abundant. 
			linear_x_ = 0.0f;
			return linear_x_;
		}
	}
	return linear_x_;
}

float VelocityProvider::getRot(){
	return angular_z_;
}

void VelocityProvider::setVel(float f){
	// Expect input f in range [0 .. 100]. Map to physical range [0 .. PHYS_MAX_LINEAR].
	if(f > INPUT_MAX_LINEAR) f = INPUT_MAX_LINEAR;
	if(f < INPUT_MIN_LINEAR) f = INPUT_MIN_LINEAR;

	// scale: physical = (input / INPUT_MAX_LINEAR) * PHYS_MAX_LINEAR
	linear_x_ = (f / INPUT_MAX_LINEAR) * PHYS_MAX_LINEAR;
}

void VelocityProvider::setRot(float f){
	// Expect input f in range [-100 .. 100]. Map to physical range [-PHYS_MAX_ROT .. PHYS_MAX_ROT].
	if(f > INPUT_MAX_ROT) f = INPUT_MAX_ROT;
	if(f < INPUT_MIN_ROT) f = INPUT_MIN_ROT;

	// scale: physical = (input / INPUT_MAX_ROT) * PHYS_MAX_ROT
	angular_z_ = (f / INPUT_MAX_ROT) * PHYS_MAX_ROT;
}

void VelocityProvider::driveForDuration(int duration, float velocity){
	// velocity is given in input range [0 .. 100], map to physical
	if(velocity > INPUT_MAX_LINEAR) velocity = INPUT_MAX_LINEAR;
	if(velocity < INPUT_MIN_LINEAR) velocity = INPUT_MIN_LINEAR;
	durationVelocity = (velocity / INPUT_MAX_LINEAR) * PHYS_MAX_LINEAR;
	end_time_ = std::chrono::steady_clock::now() + std::chrono::seconds(duration);
	state_ = State::DURATION;
}

