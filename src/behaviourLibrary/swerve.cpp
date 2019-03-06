#include "swerve.h"

Swerve::Swerve(Direction turnDir, int avoid, int standoff, int turnSpeed)
    : ArAction("Swerve", "Behaviour to avoid obstacles."),
      turnDir(turnDir),
      turnSpeed(turnSpeed),
      avoid(avoid),
      standoff(standoff)
{
}

ArActionDesired *Swerve::fire(ArActionDesired currentDesired)
{
    // reset the actionDesired (must be done)
    actionDesired.reset();

	// set up parameters
    int radius = (int)myRobot->getRobotRadius();

	double x, y, strength;

    ArPose rpose = myRobot->getPose();
    obstacleDist = myRobot->checkRangeDevicesCurrentBox(0, -radius,
                        standoff, radius, &rpose);

    avoiding = obstacleDist < standoff;
    if (avoiding)	// we've found an obstacle in front
    {
        x = (double)(obstacleDist - avoid);
		if (x < 0) x = 0;
        y = (double)(standoff - avoid);
		// get stronger as we get closer
        if (y != 0){
            strength = 1.0 - (x/y);
        }else {
            strength = 0;
        }
        actionDesired.setRotVel(turnSpeed * turnDir, strength);
        actionDesired.setVel(0.0, strength);
    }
    return &actionDesired;
}

int Swerve::getTurnSpeed() const
{
    return turnSpeed;
}

void Swerve::setTurnSpeed(int value)
{
    turnSpeed = value;
}

Swerve::Direction Swerve::getTurnDir() const
{
    return turnDir;
}

void Swerve::setTurnDir(const Direction &value)
{
    turnDir = value;
}

int Swerve::getAvoidDist() const
{
    return avoid;
}

void Swerve::setAvoidDist(int value)
{
    avoid = value;
}

int Swerve::getObstacleDist() const
{
    return obstacleDist;
}

int Swerve::getStandOff() const
{
    return standoff;
}

void Swerve::setStandOff(int value)
{
    standoff = value;
}

bool Swerve::isAvoiding() const
{
    return avoiding;
}
