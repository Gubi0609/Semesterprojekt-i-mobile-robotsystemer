#pragma once

#include <stdlib.h>    // For std::rand
#include <chrono>
#include <string>
#include <bits/stdc++.h>  //not used??
#include <mutex>
//#include eliasclass.cpp


class VelocityProvider {
public:
	VelocityProvider();
    virtual ~VelocityProvider() = default;
    
    float getVel(); //return linear x
    float getRot(); //return angular z
    void setVel(float f);
    void setRot(float f);
    void checkDurationExpiry(); 

    enum class State {IDLE, DURATION};
    State state() const {return state_;}

    void update(); //update state
    void setState(State s);

    bool getEnableDriving();
    void setEnableDriving(bool b);

    void validatingSpeeds();
    void startDuration(int s, float linear); //method to be called from receiver.cpp
    void updatePrevValues();

    void setCustomDuration(int seconds);

    void driveForDuration(int s, float linear);

    int getPreFunc();
    void updatePreFunc(int p);

private:
    mutable std::mutex mu_; //creates a mutex object
    float linear_x_;
    float angular_z_;

    //dont know if these are usable /relevant
    //bool timeStampSet = false;  
    bool enableDriving = false;

    //State data
    State state_ = State::IDLE;
    float durationVelocity = 0.0f;
    std::chrono::steady_clock::time_point end_time_;

    float prev_linear_x_ = 0.0f;
    float prev_angular_z_ = 0.0f;
    float prev_duration = 0.0f;
    State prev_state_ = State::IDLE;
    int prev_custom_duration = 0;
    int customDuration = 0;

    int presetFunctionality = 0;
    
};
