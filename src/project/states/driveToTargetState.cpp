#include "driveToTargetState.h"
#include "../config.h"

DriveToTargetState::DriveToTargetState(DefaultRobotServer *server) : pathTask(server), pathPlanningIsRunning(false)
{
}

ArActionDesired *DriveToTargetState::updateState(const ArActionDesired &currentDesired)
{
    myDesired.reset();

    return &myDesired;
}

void DriveToTargetState::enterState()
{
    if (pathPlanningIsRunning)
    {
        pathPlanningIsRunning = false;
        ArLog::log(ArLog::Normal, "Entering DriveToTargetState (returned from PathPlanningTask)");

        //TODO untested
        pathTask.getPathPlanAction()->deactivate();
        getRobot()->remUserTask("PathTask");

        myStateMachine->changeState(ConfigSingleton::getInstance()->returnState);
    }
    else
    {
        pathPlanningIsRunning = true;
        ArLog::log(ArLog::Normal, "Entering DriveToTargetState");

        //TODO untestet
        //TODO: can i add this multiple times? i need to be able to switch between all ArActions and PathPlanning
        pathTask.setNewTarget(ConfigSingleton::getInstance()->targetPose, ConfigSingleton::getInstance()->useRotation);
        getRobot()->addUserTask("PathTask", 0, &pathTask);
        //pathTask.getPathPlanAction()->activate();
    }
}

void DriveToTargetState::exitState()
{
    if (pathPlanningIsRunning)
    {
        ArLog::log(ArLog::Normal, "Exiting DriveToTargetState (starting PathPlaningTask)");
    }
    else
    {
        ArLog::log(ArLog::Normal, "Exiting DriveToTargetState");
    }
}