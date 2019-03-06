#include "sonarreadingboxaction.h"

#include <ArSonarDevice.h>

SonarReadingBoxAction::SonarReadingBoxAction(ArSonarDevice *sonar) :
    ArAction("SonarReadingBoxAction",
             "This action demonstrates the use of an SonarReadingBox."),
    sonar(sonar),
    initialized(false)
{
}

ArActionDesired *SonarReadingBoxAction::fire(ArActionDesired currentDesired)
{
    if(!sonar) {
        ArLog::log(ArLog::Normal,
                   "SonarReadingBoxAction requires a sonar device!");
        deactivate();
        return 0;
    }

    if (!initialized) {
        init();
    }
    unsigned int maxRange = sonar->getMaxRange();
    double distance = sonar->currentReadingBox(x1, y1, x2, y2, &readingPose);

    ArLog::log(ArLog::Normal, "SonarReadingBox: (%5.0f, %5.0f, %5.0f, %5.0f)",
               x1, y1, x2, y2);
    ArLog::log(ArLog::Normal,
               "Position: (%5.0f, %5.0f), Distance: %5.0f, Range: %5.0d",
               readingPose.getX(), readingPose.getY(),
               distance, maxRange);

    return 0;
}

void SonarReadingBoxAction::init()
{
    double robotRadius = myRobot->getRobotRadius();
    double boxLength = 5000;
    double boxWidth = 2 * (robotRadius + 100);

    x1 = robotRadius;
    y1 = -boxWidth * 0.5;
    x2 = x1 + boxLength;
    y2 = y1 + boxWidth;

    initialized = true;
}
