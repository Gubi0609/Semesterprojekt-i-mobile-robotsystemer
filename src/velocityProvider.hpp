#pragma once

#include <stdlib.h>    // For std::rand

class VelocityProvider {
public:
    virtual ~VelocityProvider() = default;
    virtual float getVel() = 0; //return linear x
    virtual float getRot() = 0; //return angular z
};