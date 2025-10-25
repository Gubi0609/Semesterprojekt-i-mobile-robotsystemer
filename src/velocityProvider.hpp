#pragma once

#include <stdlib.h>    // For std::rand

class VelocityProvider {
public:
    virtual ~VelocityProvider() = default;
    virtual float getVel() = 0; //return linear x
    virtual float getRot() = 0; //return angular z
};

//OtherClass declared so other transllation units can create it
class OtherClass : public VelocityProvider {
public:
	OtherClass();
	float getVel() override;
	float getRot() override;
private:
	float linear_x_;
	float angular_z_;
};
