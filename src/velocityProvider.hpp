#pragma once

#include <stdlib.h>    // For std::rand
#include <chrono>


class VelocityProvider {
public:
	VelocityProvider();
    virtual ~VelocityProvider() = default;
    float getVel(); //return linear x
    float getRot(); //return angular z
    void setVel(float f);
    void setRot(float f);
    void driveForDuration(int d, float v); //takes duration as input parameter.

    enum class State {IDLE, DURATION};
    State state() const {return state_;}

private:
    float linear_x_;
    float angular_z_;
    time_t time_stamp;
    bool timeStampSet = false;

    //State data
    State state_ = State::IDLE;
    float durationVelocity = 0.0f;
    std::chrono::steady_clock::time_point end_time_;
};
