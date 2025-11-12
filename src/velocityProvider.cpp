#include "velocityProvider.hpp"
#include <cstdlib>
#include <algorithm> // for std::clamp

VelocityProvider::VelocityProvider()
: linear_x_(0.0f), angular_z_(0.0f), state_(State::IDLE), durationVelocity(0.0f)
{}

float VelocityProvider::getVel() {
	//std::lock_guard<std::mutex> lk(mu_); //uses mutex in every method to prevent errors. lockGuard is destructed when out of scope (code exits method)
	linear_x_= std::clamp(linear_x_,0.0f, 0.22f);
	return linear_x_;
}

float VelocityProvider::getRot(){
	//std::lock_guard<std::mutex> lk(mu_);
	angular_z_= std::clamp(angular_z_,-2.84f, 2.84f);
	return angular_z_;
}

void VelocityProvider::setVel(float f){
	//std::lock_guard<std::mutex> lk(mu_);
	// Expect input f in range [0 .. 100]. Map to physical range [0 .. PHYS_MAX_LINEAR].
	if(f > INPUT_MAX_LINEAR) f = INPUT_MAX_LINEAR;
	if(f < INPUT_MIN_LINEAR) f = INPUT_MIN_LINEAR;

	// scale: physical = (input / INPUT_MAX_LINEAR) * PHYS_MAX_LINEAR
	linear_x_ = (f / INPUT_MAX_LINEAR) * PHYS_MAX_LINEAR;
}

void VelocityProvider::setRot(float f){
	//std::lock_guard<std::mutex> lk(mu_);
	// Expect input f in range [-100 .. 100]. Map to physical range [-PHYS_MAX_ROT .. PHYS_MAX_ROT].
	if(f > INPUT_MAX_ROT) f = INPUT_MAX_ROT;
	if(f < INPUT_MIN_ROT) f = INPUT_MIN_ROT;

	// scale: physical = (input / INPUT_MAX_ROT) * PHYS_MAX_ROT
	angular_z_ = (f / INPUT_MAX_ROT) * PHYS_MAX_ROT;
}

void VelocityProvider::checkDurationExpiry(){ //maybe remove this and use startDuration instead
	//std::lock_guard<std::mutex> lk(mu_);
	if(!(std::chrono::steady_clock::now() <end_time_)){
		linear_x_ = 0.0f;
		angular_z_ = 0.0f;
		state_ = State::IDLE;
	}
}

void VelocityProvider::startDuration(float seconds){
	//i dont think this is used
	//std::lock_guard<std::mutex> lk(mu_);
	customDuration = seconds; //just to track duration and prev duration
	end_time_ = std::chrono::steady_clock::now() + 
		std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(seconds));
	state_ = State::DURATION;
}


void VelocityProvider::update(){
	//std::lock_guard<std::mutex> lk(mu_);
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

			//indsæt logik til at checke at values nu ikke er lig prev values.
			if(prev_angular_z_ !=angular_z_ || prev_linear_x_ != linear_x_ || prev_state_ != state_ || customDuration != prev_custom_duration){
				updatePrevValues();
				end_time_ = std::chrono::steady_clock::now() +
					std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(customDuration));

			}

			//bit to command reader fra recieve currentcommand...
			//velocity = currentSignal //linear_X 		//vi læser data fra anden fil
			//duration = ???
			//curentSignal
			//end_time_ = std::chrono::steady_clock::now() + std::chrono::seconds(duration);
			checkDurationExpiry();
	}			
}

void VelocityProvider::setState(State state){
	//std::lock_guard<std::mutex> lk(mu_);
	state_ = state;
}

void VelocityProvider::driveForDuration(float seconds, float linear){
	//std::lock_guard<std::mutex> lk(mu_);
	state_ = State::DURATION;
	setVel(linear);
	customDuration = seconds;
}

void VelocityProvider::turnForDuration(float seconds, float rotational){
	//std::lock_guard<std::mutex> lk(mu_);
	state_ = State::DURATION;
	setRot(rotational);
	customDuration = seconds;
}

void VelocityProvider::setEnableDriving(bool b) {
	//std::lock_guard<std::mutex> lk(mu_);
	enableDriving = b;
}

bool VelocityProvider::getEnableDriving(){
	//std::lock_guard<std::mutex> lk(mu_);
	return enableDriving;	
}

void VelocityProvider::updatePrevValues(){
	//std::lock_guard<std::mutex> lk(mu_);
	prev_angular_z_ = angular_z_;
	prev_linear_x_ = linear_x_;
	prev_state_ = state_;
	prev_custom_duration = customDuration;
}

void VelocityProvider::setCustomDuration(float s){
	//std::lock_guard<std::mutex> lk(mu_);
	customDuration = s;
}

int VelocityProvider::getPreFunc(){
	return presetFunctionality;
}

void VelocityProvider::updatePreFunc(int p){
	presetFunctionality = p;
}