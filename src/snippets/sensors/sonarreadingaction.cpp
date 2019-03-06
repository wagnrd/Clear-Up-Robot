#include "sonarreadingaction.h"

#include <ArRobot.h>

SonarReadingAction::SonarReadingAction() :
    ArAction("SonarReadingAction",
             "This action prints all sonar readings.")
{
}

ArActionDesired *SonarReadingAction::fire(ArActionDesired currentDesired)
{
    if (!myRobot) {
        ArLog::log(ArLog::Normal,
                   "Error: SonarReadingAction requires a robot!");
        deactivate();
        return 0;
    }

    for (int i = 0; i < myRobot->getNumSonar(); ++i) {
        ArSensorReading *reading = myRobot->getSonarReading(i);
        if (reading) {
            ArPose pose = reading->getPose();
            ArLog::log(ArLog::Normal,
                       "Sonar: %2d, Reading: (%5.0f, %5.0f), Distance: %5d",
                       i, pose.getX(), pose.getY(),
                       reading->getRange());
        }
    }
    return 0;
}
