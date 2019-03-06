#include "sonarrangeaction.h"

#include <ArSonarDevice.h>

SonarRangeAction::SonarRangeAction() :
    ArAction("SonarRangeAction",
             "This action prints all sonar range results.")
{
}

ArActionDesired *SonarRangeAction::fire(ArActionDesired currentDesired)
{
    if (!myRobot) {
        ArLog::log(ArLog::Terse, "Error: SonarRangeAction requires a robot!");
        deactivate();
        return 0;
    }

    int distance;
    for (int i = 0; i < myRobot->getNumSonar(); ++i) {
        distance = myRobot->getSonarRange(i);
        ArLog::log(ArLog::Normal, "Sonar: %2d, Distance: %5d", i, distance);
    }

    return 0;
}
