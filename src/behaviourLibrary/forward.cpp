#include "forward.h"

Forward::Forward(double speed) :
    ArAction("Forward", "Behaviour to move at constant velocity."),
    speed(speed)
{
}

ArActionDesired *Forward::fire(ArActionDesired currentDesired)
{
	myDesired.reset();
    myDesired.setVel(speed);
    return &myDesired;
}

double Forward::getSpeed() const
{
    return speed;
}

void Forward::setSpeed(double value)
{
    speed = value;
}
