#include <Aria.h>

class Forward : public ArAction
{
public:
    Forward(double speed=500);
    virtual ArActionDesired *fire(ArActionDesired currentDesired);

    double getSpeed() const;
    void setSpeed(double value);

protected:
    ArActionDesired myDesired; // what the action wants to do
    double speed;  // how fast to go
};
