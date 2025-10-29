#pragma once

#include <stdlib.h>    // For std::rand

class VelocityProvider {
public:
    virtual ~VelocityProvider() = default;
    float getVel(); //return linear x
    float getRot(); //return angular z
    void setVel(float f);
    void setRot(float f);
private:
    float linear_x_;
    float angular_z_;
};