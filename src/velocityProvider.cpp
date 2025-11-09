#include "velocityProvider.hpp"
#include <cstdlib>

VelocityProvider::VelocityProvider()
: linear_x_(0.0f), angular_z_(0.0f), state_(State::IDLE), durationVelocity(0.0f)
{}

float VelocityProvider::getVel() {
	std::lock_guard<std::mutex> lk(mu_); //uses mutex in every method to prevent errors. lockGuard is destructed when out of scope (code exits method)
	linear_x_= std::clamp(linear_x_,0.0f, 0.22f);
	return linear_x_;
}

float VelocityProvider::getRot(){
	std::lock_guard<std::mutex> lk(mu_);
	angular_z_= std::clamp(angular_z_,-2.84f, 2.84f);
	return angular_z_;
}

void VelocityProvider::setVel(float f){
	std::lock_guard<std::mutex> lk(mu_);
	linear_x_ = std::clamp(f, 0.0f, 0.22f);
}

void VelocityProvider::setRot(float f){
	std::lock_guard<std::mutex> lk(mu_);
	angular_z_ = std::clamp(f, -2.84f, 2.84f);
}
 
void VelocityProvider::checkDurationExpiry(){ //maybe remove this and use startDuration instead
	std::lock_guard<std::mutex> lk(mu_);
	if(!(std::chrono::steady_clock::now() <end_time_)){
		linear_x_ = 0.0f;
		angular_z_ = 0.0f;
		state_ = State::IDLE;
	}
}

void VelocityProvider::startDuration(int seconds, float linear_vel){
	std::lock_guard<std::mutex> lk(mu_);
	linear_x_ = std::clamp(linear_vel, 0.0f, 0.22f);
	angular_z_ = 0.0f;
	end_time_ = std::chrono::steady_clock::now() + std::chrono::seconds(seconds);
	state_ = State::DURATION;
}


void VelocityProvider::update(){
	std::lock_guard<std::mutex> lk(mu_);
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
			checkDurationExpiry();
	}			
}

void VelocityProvider::setState(State state){
	std::lock_guard<std::mutex> lk(mu_);
	state_ = state;
}

void VelocityProvider::setEnableDriving(bool b) {
	std::lock_guard<std::mutex> lk(mu_);
	enableDriving = b;
}

bool VelocityProvider::getEnableDriving(){
	std::lock_guard<std::mutex> lk(mu_);
	return enableDriving;	
}