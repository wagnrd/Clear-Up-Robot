#include "robotinfoaction.h"

#include <ArRobot.h>

RobotInfoAction::RobotInfoAction() :
    ArAction("RobotInfoAction",
             "This action prints the pose and radius of the robot.")
{
}

ArActionDesired *RobotInfoAction::fire(ArActionDesired currentDesired)
{
    double x,y,th, radius;

    x = myRobot->getX();
    y = myRobot->getY();
    th = myRobot->getTh();
    radius = myRobot->getRobotRadius();

    ArLog::log(ArLog::Normal,
               "Pose: (%5.0fmm, %5.0fmm, %3.2f°), Radius: %5.0f)",
               x, y, th, radius);

    return 0;
}
