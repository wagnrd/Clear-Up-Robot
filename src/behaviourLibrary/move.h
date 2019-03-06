#ifndef MOVE_H
#define MOVE_H

#include <ArAction.h>

class Move : public ArAction
{
public:
    Move(double distance=1000, double breakDist=200,
         double minSpeed=50, double maxSpeed=500);
    virtual ArActionDesired *fire(ArActionDesired currentDesired);

    virtual void activate();

    double getDistance() const;
    void setDistance(double value);

    double getBreakDist() const;
    void setBreakDist(double value);

    double getMinSpeed() const;
    void setMinSpeed(double value);

    double getMaxSpeed() const;
    void setMaxSpeed(double value);

protected:
    ArActionDesired actionDesired; // what the action wants to do
    double distance;  // distanct to drive
    double breakDist; // distance for deceleration
    double minSpeed;  // minimal speed
    double maxSpeed;  // maximal speed
    double startDist; // odometer distance on start
};

#endif // MOVE_H
