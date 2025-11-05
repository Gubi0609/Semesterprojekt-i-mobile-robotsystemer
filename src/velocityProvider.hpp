#pragma once

#include <stdlib.h>    // For std::rand
#include <chrono>
#include <string>
#include <bits/stdc++.h>
//#include eliasclass.cpp


class VelocityProvider {
public:
	VelocityProvider();
    virtual ~VelocityProvider() = default;
    
    float getVel(); //return linear x
    float getRot(); //return angular z
    void setVel(float f);
    void setRot(float f);
    void driveForDuration(); //takes duration as input parameter.

    enum class State {IDLE, DURATION};
    State state() const {return state_;}

    void update(); //update state
    void setState(State s);

    bool getEnableDriving();
    void setEnableDriving(bool b);

    void validatingSpeeds();

private:
    float linear_x_;
    float angular_z_;
    time_t time_stamp;
    bool timeStampSet = false;

    bool enableDriving = false;

    //State data
    State state_ = State::IDLE;
    float durationVelocity = 0.0f;
    std::chrono::steady_clock::time_point end_time_;
};
