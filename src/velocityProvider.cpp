#include "velocityProvider.hpp"
#include <cstdlib>

VelocityProvider::VelocityProvider()
: linear_x_(0.0f), angular_z_(0.0f), state_(State::IDLE), durationVelocity(0.0f)
{}

float VelocityProvider::getVel() {
	linear_x_= std::clamp(linear_x_,0.0f, 0.22f);
	return linear_x_;
}

float VelocityProvider::getRot(){
	angular_z_= std::clamp(angular_z_,-2.84f, 2.84f);
	return angular_z_;
}



void VelocityProvider::setRot(float f){
	angular_z_ = f;
}

void VelocityProvider::driveForDuration(){
	if(!(std::chrono::steady_clock::now() <end_time_)){
		linear_x_ = 0.0f;
		angular_z_ = 0.0f;
		state_ = State::IDLE;
	}
}

void VelocityProvider::update(){
	// if(!enableDriving()) return;
	if(state_ == State::IDLE){
		//switch(receiver.get(currentSignal)):  //currentSignal har værdier.
		// case : 255:
		//	state_ = State::IDLE;
		


		if(angular_z_!= 0.0f || linear_x_ != 0.0f){
			angular_z_ = 0.0f;
			linear_x_ = 0.0f;
		}
	} else if(state_ == State::DURATION){ //kan med fordel bruge switch (godt)
			//bit to command reader fra recieve currentcommand...
			//velocity = currentSignal //linear_X 		//vi læser data fra anden fil
			//duration = ???
			//curentSignal
			//end_time_ = std::chrono::steady_clock::now() + std::chrono::seconds(duration);
			driveForDuration();
	}			
}

void VelocityProvider::setState(State state){
	state_ = state;
}

void VelocityProvider::setEnableDriving(bool b) {
	enableDriving = b;
}

bool VelocityProvider::getEnableDriving(){
	return enableDriving;	
}