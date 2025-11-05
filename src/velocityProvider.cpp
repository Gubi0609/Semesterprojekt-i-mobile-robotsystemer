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
	if(f>0.22) f=0.22;
	if(f<0) f =0.0f;

	linear_x_ = f;
}

void VelocityProvider::setRot(float f){
	angular_z_ = f;
}

void VelocityProvider::driveForDuration(int duration, float velocity){
	durationVelocity = velocity;
	end_time_ = std::chrono::steady_clock::now() + std::chrono::seconds(duration);
	state_ = State::DURATION;
}

