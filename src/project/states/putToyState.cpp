#include "putToyState.h"
#include "../config.h"
#include "../utils/utils.hpp"

#define USE_GRIPPER

void PutToyState::enterState()
{
    ArLog::log(ArLog::Normal, "Entering PutToyState");

    //check if last path planning was successfull (= we are at our correct position)
    if (!ConfigSingleton::getInstance()->lastPathPlanningSuccess)
    {
        ArLog::log(ArLog::Terse, "Last path planing was not successfull. Returning to home");

        myStateMachine->changeState(MainStateMachine::Action::GO_TO_HOME);
        return;
    }

    gripper = ConfigSingleton::getInstance()->gripper;
    currentSubState = SubState::INITIAL_STATE;
    waitingCounter = 0;
}

ArActionDesired *PutToyState::updateState(const ArActionDesired &currentDesired)
{
#ifndef USE_GRIPPER
    ArLog::log(ArLog::Normal, "No gripper to operate!");
#endif

#ifdef USE_GRIPPER
    switch (currentSubState)
    {
    // assuming the robot currently has the ball and stands in front of the toy box
    case SubState::INITIAL_STATE:
        ArLog::log(ArLog::Normal, "INITIAL_STATE");
        if (Utils::goUntilObstacleIn(myDesired, objectsInBox * distanceBetweenToys + distanceToWall, 100))
        {
            currentSubState = SubState::ROBOT_IN_DEPLOY_POS;
            ++objectsInBox;
        }

        break;

    case SubState::ROBOT_IN_DEPLOY_POS:
        ArLog::log(ArLog::Normal, "ROBOT_IN_DEPLOY_POS");
        Utils::gripperLiftDown();

        if (waitingCounter > 0 && !gripper->isLiftMoving())
        {
            currentSubState = SubState::TOY_ON_FLOOR;
            waitingCounter = 0;
        }
        else
        {
            waitingCounter++;
        }

        break;

    case SubState::TOY_ON_FLOOR:
        ArLog::log(ArLog::Normal, "TOY_ON_FLOOR");
        Utils::gripperOpen();

        if (waitingCounter > 0 && !gripper->isGripMoving())
        {
            currentSubState = SubState::GRIPPER_OPEN;
            waitingCounter = 0;
        }
        else
        {
            waitingCounter++;
        }

        break;

    case SubState::GRIPPER_OPEN:
        ArLog::log(ArLog::Normal, "GRIPPER_OPEN");
        if (Utils::goUntilObstacleGoneIn(myDesired, 900, -130))
        {
            currentSubState = SubState::ROBOT_IN_SAVE_POS;
            Utils::gripperStore();
        }
        else
        {
            // ReadingBox behind the robot to avoid hitting dynamic objects
            ArSonarDevice *sonar = ConfigSingleton::getInstance()->sonar;
            double robotRadius = getRobot()->getRobotRadius();
            double allowedDistance = robotRadius + 200;
            double x1 = -(robotRadius);
            double y1 = robotRadius + 100;
            double x2 = -5000;
            double y2 = -(robotRadius + 100);

            double distance = sonar->currentReadingBox(x1, y1, x2, y2);

            if (distance < allowedDistance)
            {
                myDesired.setVel(0);
            }
        }

        break;

    case SubState::ROBOT_IN_SAVE_POS:
        ArLog::log(ArLog::Normal, "ROBOT_IN_SAVE_POS");
        myStateMachine->changeState(MainStateMachine::Action::GO_TO_PLAY_AREA);

        break;
    }
#endif

    return &myDesired;
}

void PutToyState::exitState()
{
    ArLog::log(ArLog::Normal, "Exiting PutToyState");
}