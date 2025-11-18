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
    // Input and physical limits
    // Users should provide linear velocity in range [-100 .. 100]
    // which will be mapped to physical linear [-PHYS_MAX_LINEAR .. PHYS_MAX_LINEAR]
    static constexpr float INPUT_MIN_LINEAR = -100.0f;
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
    int prev_custom_duration = 0;
    int customDuration = 0;

    int presetFunctionality = 0;
    
};

// VelocityController: keyboard (WASD) based controller that injects speeds into a VelocityProvider.
// Usage: create a VelocityController with a pointer to an existing VelocityProvider and call
// start() to run the background input loop, stop() to stop. The controller ramps speed from
// 0..100 over approx 1.5s while keys are held.
class VelocityController {
public:
    explicit VelocityController(VelocityProvider* provider);
    ~VelocityController();

    // Start/stop the background thread. Non-blocking start.
    void start();
    void stop();
    bool isRunning() const { return running_; }

    // Optional: configure ramp duration (seconds to go from 0 to 100)
    void setRampSeconds(float seconds) { ramp_seconds_ = seconds; }

private:
    void runLoop();

    VelocityProvider* provider_ = nullptr;
    std::atomic<bool> running_{false};
    std::thread loop_thread_;
    float ramp_seconds_ = 1.5f; // default ramp time
};
