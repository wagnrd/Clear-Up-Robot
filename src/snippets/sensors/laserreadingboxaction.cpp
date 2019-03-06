#include "laserreadingboxaction.h"

#include <ArLaser.h>
#include <ArRobot.h>

LaserReadingBoxAction::LaserReadingBoxAction(ArLaser *laser) :
    ArAction("LaserReadingBoxAction",
             "This action demonstrates the usage of an laser reading box."),
    laser(laser),
    initialized(false)
{
}

void LaserReadingBoxAction::activate()
{
    ArLog::log(ArLog::Normal, "LaserReadingBox activate()!");
    if (!myRobot) {
        ArLog::log(ArLog::Terse,
                   "Error: LaserReadingBoxAction requires a robot!");
        deactivate();
    } else if (!laser) {
        ArLog::log(ArLog::Terse,
                   "Error: LaserReadingBoxAction requires a laser!");
        deactivate();
    } else {
        init();

        ArAction::activate();
    }
}

void LaserReadingBoxAction::init()
{
    double robotRadius = myRobot->getRobotRadius();
    double boxLength = 10000;
    double boxWidth = 2 * (robotRadius + 100);

    x1 = robotRadius;
    y1 = -boxWidth * 0.5;
    x2 = x1 + boxLength;
    y2 = y1 + boxWidth;

    initialized = true;
}

ArActionDesired *LaserReadingBoxAction::fire(ArActionDesired currentDesired)
{
    if(!initialized) {
        init();
    }

    laser->lockDevice();

    if (laser->isConnected()) {
        unsigned int maxRange = laser->getMaxRange();
        double distance = laser->currentReadingBox(x1, y1, x2, y2,
                                                   &reading);
        ArLog::log(ArLog::Normal,
                   "LaserReadingBox: (%5.0f, %5.0f, %5.0f, %5.0f)",
                   x1,y1,x2,y2);
        ArLog::log(ArLog::Normal,
                   "Pos: (%5.0f, %5.0f), Dist: %5.0f, Range: %5.0d",
                   reading.getX(), reading.getY(),
                   distance, maxRange);

    } else if (laser->isTryingToConnect()) {
        ArLog::log(ArLog::Normal, "Connecting to Laser %s, please wait.",
                   laser->getName());
    } else {
        laser->asyncConnect();
    }

    laser->unlockDevice();
    return 0;
}
