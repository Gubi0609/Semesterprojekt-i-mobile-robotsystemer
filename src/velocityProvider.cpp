#include "velocityProvider.hpp"
#include <cstdlib>
#include <algorithm> // for std::clamp

VelocityProvider::VelocityProvider()
: linear_x_(0.0f), angular_z_(0.0f), state_(State::IDLE), durationVelocity(0.0f)
{}

float VelocityProvider::getVel() {
	//use recursive mutex instead to avoid deadlock, but still be able to use mutex in all methods
	//std::lock_guard<std::mutex> lk(mu_); //uses mutex in every method to prevent errors. lockGuard is destructed when out of scope (code exits method)
	std::lock_guard<std::recursive_mutex> lk(remu_);
	linear_x_= std::clamp(linear_x_,0.0f, 0.22f);
	return linear_x_;
}

float VelocityProvider::getRot(){
	std::lock_guard<std::recursive_mutex> lk(remu_);
	angular_z_= std::clamp(angular_z_,-2.84f, 2.84f);
	return angular_z_;
}


void VelocityProvider::setVel(float f){
	std::lock_guard<std::recursive_mutex> lk(remu_);
	// Expect input f in range [0 .. 100]. Map to physical range [0 .. PHYS_MAX_LINEAR].
	if(f > INPUT_MAX_LINEAR) f = INPUT_MAX_LINEAR;
	if(f < INPUT_MIN_LINEAR) f = INPUT_MIN_LINEAR;

	// scale: physical = (input / INPUT_MAX_LINEAR) * PHYS_MAX_LINEAR
	linear_x_ = (f / INPUT_MAX_LINEAR) * PHYS_MAX_LINEAR;
}

void VelocityProvider::setRot(float f){
	std::lock_guard<std::recursive_mutex> lk(remu_);
	// Expect input f in range [-100 .. 100]. Map to physical range [-PHYS_MAX_ROT .. PHYS_MAX_ROT].
	if(f > INPUT_MAX_ROT) f = INPUT_MAX_ROT;
	if(f < INPUT_MIN_ROT) f = INPUT_MIN_ROT;

	// scale: physical = (input / INPUT_MAX_ROT) * PHYS_MAX_ROT
	angular_z_ = (f / INPUT_MAX_ROT) * PHYS_MAX_ROT;
}

void VelocityProvider::checkDurationExpiry(){
	std::lock_guard<std::recursive_mutex> lk(remu_);
	if(!(std::chrono::steady_clock::now() <end_time_)){
		linear_x_ = 0.0f;
		angular_z_ = 0.0f;
		state_ = State::IDLE;
	}
}


void VelocityProvider::update(){
	std::lock_guard<std::recursive_mutex> lk(remu_);
	// if(!enableDriving()) return;
	if(state_ == State::IDLE){
		//switch(receiver.get(currentSignal)):  //currentSignal har værdier.
		// case : 255:
		//	state_ = State::IDLE;

		if(angular_z_!= 0.0f || linear_x_ != 0.0f){
			angular_z_ = 0.0f;
			linear_x_ = 0.0f;
		}
		// Reset previous values when IDLE so duplicate commands work
		prev_angular_z_ = 0.0f;
		prev_linear_x_ = 0.0f;
		prev_custom_duration = 0.0;
		prev_state_ = State::IDLE;
	} else if(state_ == State::DURATION){ //kan med fordel bruge switch (godt)

			//indsæt logik til at checke at values nu ikke er lig prev values.
			if(updateDurationValues){
				updateDurationValues = false;
				end_time_ = std::chrono::steady_clock::now() +
					std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(customDuration));
			}
			checkDurationExpiry();
	} else if (state_ == State::CONTINUOUS)
	{
		prev_state_ = state_;
		//logging here? maybe
	}
	
}

void VelocityProvider::setState(State state){
	std::lock_guard<std::recursive_mutex> lk(remu_);
	state_ = state;
}

void VelocityProvider::forwardForDuration(float seconds, float linear){
	std::lock_guard<std::recursive_mutex> lk(remu_);
	state_ = State::DURATION;
	setVel(linear);
	customDuration = seconds;
	updateDurationValues = true;
}

void VelocityProvider::driveForDuration(float seconds, float lin, float rot){
	std::lock_guard<std::recursive_mutex> lk(remu_);
	state_ = State::DURATION;
	customDuration = seconds;
	auto[adjustedLin, adjustedRot] = adjustLinAndRot(lin, rot);
	setVel(adjustedLin);
	setRot(adjustedRot);
	updateDurationValues = true;
}

void VelocityProvider::driveContinuous(float lin){
	std::lock_guard<std::recursive_mutex> lk(remu_);
	state_ = State::CONTINUOUS;
	auto[adjustedLin, adjustedRot] = adjustLinAndRot(lin,0.0f);
	setVel(adjustedLin);
	setRot(adjustedRot);
}

std::tuple<float, float> VelocityProvider::adjustLinAndRot(float lin, float rot){
	if(lin+std::fabs(rot)<100.0f){
		return {lin,rot};
	}
	while(lin+std::fabs(rot)>100.0f){
		lin = lin*0.95f;
		rot = rot*0.9f;
	}
	return {lin,rot};
}

void VelocityProvider::turnForDuration(float seconds, float rotational){
	std::lock_guard<std::recursive_mutex> lk(remu_);
	state_ = State::DURATION;
	setRot(rotational);
	customDuration = seconds;
	updateDurationValues = true;
}

void VelocityProvider::setEnableDriving(bool b) {
	//std::lock_guard<std::mutex> lk(mu_);
	enableDriving = b;
}

bool VelocityProvider::getEnableDriving(){
	//std::lock_guard<std::mutex> lk(mu_);
	return enableDriving;	
}

//no mutex because this is inner fucntion
void VelocityProvider::updatePrevValues(){
	//only used as helper function. If called from other file on its own, add recursive mutex
	prev_angular_z_ = angular_z_;
	prev_linear_x_ = linear_x_;
	prev_state_ = state_;
	prev_custom_duration = customDuration;
}

void VelocityProvider::setCustomDuration(float s){
	std::lock_guard<std::recursive_mutex> lk(remu_);
	customDuration = s;
}

int VelocityProvider::getPreFunc(){
	std::lock_guard<std::recursive_mutex> lk(remu_);
	return presetFunctionality;
}

void VelocityProvider::updatePreFunc(int p){
	std::lock_guard<std::recursive_mutex> lk(remu_);
	presetFunctionality = p;
}