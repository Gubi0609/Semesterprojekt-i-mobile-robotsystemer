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

    //State data
    State state_ = State::IDLE;
    float durationVelocity = 0.0f;
    std::chrono::steady_clock::time_point end_time_;
};
