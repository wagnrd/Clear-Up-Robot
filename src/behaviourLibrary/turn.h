#ifndef TURN_H
#define TURN_H

#include <ArAction.h>

class Turn : public ArAction
{
public:
    Turn(double heading, double delta=1);
    virtual ArActionDesired *fire(ArActionDesired currentDesired);

    virtual void activate();

    double getHeading() const;
    void setHeading(double value);

    double getDelta() const;
    void setDelta(double value);

    double normalizeAngle(double angle);

protected:
    ArActionDesired actionDesired; // what the action wants to do
    double heading; // the relative rotation to be performed
    double startTh; // the robots heading on start
    double delta;   // the error to be accepted
};

#endif // TURN_H
