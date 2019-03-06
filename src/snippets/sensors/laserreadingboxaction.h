#ifndef LASERREADINGBOX_H
#define LASERREADINGBOX_H

#include <ArAction.h>

class LaserReadingBoxAction : public ArAction
{
public:
    LaserReadingBoxAction(ArLaser *laser);
    ArActionDesired *fire(ArActionDesired currentDesired);

    virtual void activate();

protected:
    ArLaser *laser;
    ArPose reading;
    double x1, x2, y1, y2;

    void init();
    bool initialized;
};

#endif // LASERREADINGBOX_H
