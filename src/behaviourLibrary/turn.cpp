#include "turn.h"

#include <ArRobot.h>

Turn::Turn(double heading, double delta) :
    ArAction("Turn", "Rotates the robot by a given angle."),
    heading(heading),
    delta(delta)
{
}

ArActionDesired *Turn::fire(ArActionDesired currentDesired)
{
    actionDesired.reset();

    double th = normalizeAngle(myRobot->getPose().getTh());
    double targetTh = normalizeAngle(startTh + heading);

    actionDesired.setHeading(targetTh);

    if(abs(th - targetTh) < delta){
        deactivate();
    }

    return &actionDesired;
}

void Turn::activate()
{
    startTh = myRobot->getPose().getTh();
    ArAction::activate();
}

double Turn::getHeading() const
{
    return heading;
}

void Turn::setHeading(double value)
{
    heading = value;
}

double Turn::getDelta() const
{
    return delta;
}

void Turn::setDelta(double value)
{
    delta = value;
}

double Turn::normalizeAngle(double angle)
{
    int angleI = (int) angle;
    double decimal = angle - angleI;
    return (angleI+360)%360 + decimal;
}
