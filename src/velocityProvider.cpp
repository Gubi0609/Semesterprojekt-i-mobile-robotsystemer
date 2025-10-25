#include "velocityProvider.hpp"

//example provider implemented in another file
class OtherClass : public VelocityProvider {
public:
    OtherClass() : linear_x_(0.0f), angular_z_(0.0f) {}

    //called by node; these can update internal state
    float getVel() override {
        linear_x_ = 0.1;
        return linear_x_;
    }

    float getRot() override {
        angular_z_ = 0;
        return angular_z_;
    }

        // Make random numner between a and b
    float rand_FloatRange(float a, float b)
    {
        return ((b - a) * ((float)rand() / RAND_MAX)) + a;
    }

private:
    float linear_x_;
    float angular_z_;
};