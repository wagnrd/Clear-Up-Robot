#include "arc.h"

Arc::Arc(double vel, double rotVel) :
    ArAction("Arc",
             "Behaviour to drive an Arc."),
    vel(vel),
    rotVel(rotVel)
{
}

ArActionDesired *Arc::fire(ArActionDesired currentDesired)
{
    actionDesired.reset();

    actionDesired.setVel(vel);
    actionDesired.setRotVel(rotVel);
    return &actionDesired;
}

double Arc::getVel() const
{
    return vel;
}

void Arc::setVel(double value)
{
    vel = value;
}

double Arc::getRotVel() const
{
    return rotVel;
}

void Arc::setRotVel(double value)
{
    rotVel = value;
}
