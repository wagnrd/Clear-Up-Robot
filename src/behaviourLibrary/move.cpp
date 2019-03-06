#include "move.h"

#include <Aria.h>

Move::Move(double distance, double breakDist,
           double minSpeed, double maxSpeed) :
    ArAction("Move", "Moves a given distance"),
    distance(distance),
    breakDist(breakDist),
    minSpeed(minSpeed),
    maxSpeed(maxSpeed)
{
}

ArActionDesired *Move::fire(ArActionDesired currentDesired)
{
    actionDesired.reset();

    double distMoved = myRobot->getOdometerDistance() - startDist;

    // stop when distance moved
    if (distMoved >= distance) {
        actionDesired.setVel(0);
        deactivate();
        return 0;
    }

    // calculate speed
    double distToGoal = distance - distMoved;
    double s = (distToGoal/breakDist) * maxSpeed;
    // clamp speed
    s = (s < minSpeed ? minSpeed : (s > maxSpeed ? maxSpeed : s));

    actionDesired.setVel(s);

    return &actionDesired;
}

void Move::activate()
{
    startDist = myRobot->getOdometerDistance();
    ArAction::activate();
}

double Move::getDistance() const
{
    return distance;
}

void Move::setDistance(double value)
{
    distance = value;
}

double Move::getBreakDist() const
{
    return breakDist;
}

void Move::setBreakDist(double value)
{
    breakDist = value;
}

double Move::getMinSpeed() const
{
    return minSpeed;
}

void Move::setMinSpeed(double value)
{
    minSpeed = value;
}

double Move::getMaxSpeed() const
{
    return maxSpeed;
}

void Move::setMaxSpeed(double value)
{
    maxSpeed = value;
}
