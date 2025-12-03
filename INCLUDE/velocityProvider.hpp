#pragma once

#include <stdlib.h>    // For std::rand
#include <chrono>
#include <string>
#include <bits/stdc++.h>  
#include <mutex>

//to return multiple floats
#include <tuple>

//to use fabs(float abs)
#include <cmath>
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
    void startDuration(float s); //method to be called from receiver.cpp
    void updatePrevValues();

    void setCustomDuration(float seconds);

    void forwardForDuration(float s, float linear);
    void turnForDuration(float s, float rotational);
    void driveForDuration(float s, float linear, float rotational);

    std::tuple<float, float> adjustLinAndRot(float linear, float rotational);

    int getPreFunc();
    void updatePreFunc(int p);

private:
    //using recursive mutex
    mutable std::recursive_mutex remu_;
    //mutable std::mutex mu_; //creates a mutex object
    float linear_x_;
    float angular_z_;
    // Input and physical limits
    // Users should provide linear velocity in range [0 .. 100]
    // which will be mapped to physical linear [0 .. PHYS_MAX_LINEAR]
    static constexpr float INPUT_MIN_LINEAR = 0.0f;
    static constexpr float INPUT_MAX_LINEAR = 100.0f;
    static constexpr float PHYS_MAX_LINEAR = 0.22f;

    // Users should provide angular velocity in range [-100 .. 100]
    // which will be mapped to physical angular [-PHYS_MAX_ROT .. PHYS_MAX_ROT]
    static constexpr float INPUT_MIN_ROT = -100.0f;
    static constexpr float INPUT_MAX_ROT = 100.0f;
    static constexpr float PHYS_MAX_ROT = 2.84f;
    time_t time_stamp;
    bool timeStampSet = false;

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
    double prev_custom_duration = 0.0;
    double customDuration = 0.0;

    int presetFunctionality = 0;
    
};
