#include "goToTask.h"

#include <ArRobot.h>
#include "defaultrobotserver.h"
#include "../config.h"
#include "../stateMachine/mainStateMachine.h"

PathTask::PathTask(DefaultRobotServer *server) : robot(server->getRobot()),
                                                 pathPlanningTask(server->getPathTask()),
                                                 state(ArPathPlanningTask::NOT_INITIALIZED),
                                                 previousState(ArPathPlanningTask::NOT_INITIALIZED),
                                                 laser(server->getLaser()), newGoal(true)
{
}

void PathTask::invoke()
{
    bool logOutput = false;
    laser->lockDevice();

    if (laser->isConnected())
    {
        //Output debug data
        char text[512];
        state = pathPlanningTask->getState();
        if (state != previousState)
        {
            pathPlanningTask->getStatusString(text, sizeof(text));
            ArLog::log(ArLog::Normal, "Planning status: %s.", text);
            //ArLog::log(ArLog::Normal, "State: %s", state);
            logOutput = true;
        }
        previousState = state;

        //set new goal (only if already initialized)
        if (newGoal && state != ArPathPlanningTask::NOT_INITIALIZED)
        {
            pathPlanningTask->pathPlanToPose(goal, useRotation);

            newGoal = false;

            logOutput = true;

            laser->unlockDevice();
            return; //state = ArPathPlanningTask::MOVING_TO_GOAL; //TODO tmp to prevent instant exit
        }

        switch (state)
        {
        case ArPathPlanningTask::NOT_INITIALIZED: // Task not initialized
            if (logOutput)
            {
                ArLog::log(ArLog::Normal, "STATE: NOT_INITIALIZED");
            }
            // CONFIG
            pathPlanningTask->setPlanFreeSpace(700);
            pathPlanningTask->setFrontClearance(35);
            pathPlanningTask->setSideClearanceAtSlowSpeed(20);
            pathPlanningTask->setSideClearanceAtFastSpeed(700);
            //pathPlanningTask->setFrontPaddingAtSlowSpeed(50);
            //pathPlanningTask->setFrontPaddingAtFastSpeed(700);
            pathPlanningTask->setResistance(50);
            pathPlanningTask->setGoalDistanceTolerance(50);
            pathPlanningTask->setGoalSpeed(100);
            pathPlanningTask->init();

            /*
            ArLog::log(ArLog::Normal, "Goal dist Tolerance: %f", pathPlanningTask->getGoalDistanceTolerance()); //def: 200
            ArLog::log(ArLog::Normal, "Goal angle Tolerance: %f", pathPlanningTask->getGoalAngleTolerance());   //def: 10
            ArLog::log(ArLog::Normal, "Goal speed: %f", pathPlanningTask->getGoalSpeed());                      //def: 250
            ArLog::log(ArLog::Normal, "Goal rot speed: %f", pathPlanningTask->getGoalRotSpeed());               //def: 33
            ArLog::log(ArLog::Normal, "slow speed: %f", pathPlanningTask->getSlowSpeed());                      //def: 100
            ArLog::log(ArLog::Normal, "fast speed: %f", pathPlanningTask->getFastSpeed());                      //def: 2000

            ArLog::log(ArLog::Normal, "Goal deltaDist Tolerance: %f", pathPlanningTask->getGoalDistanceTolerance()); //def: 200
            ArLog::log(ArLog::Normal, "Resistance: %d", pathPlanningTask->getResistance());                          //def: 2
            ArLog::log(ArLog::Normal, "Front Padding Slow: %f", pathPlanningTask->getFrontPaddingAtSlowSpeed());     //def: 100
            ArLog::log(ArLog::Normal, "Front Padding Fast: %f", pathPlanningTask->getFrontPaddingAtFastSpeed());     //def: 1000
            ArLog::log(ArLog::Normal, "Front Clearance: %f", pathPlanningTask->getFrontClearance());                 //def: 100
            ArLog::log(ArLog::Normal, "Side Clearance Slow: %f", pathPlanningTask->getSideClearanceAtSlowSpeed());   //def: 100
            ArLog::log(ArLog::Normal, "Side Clearance Fast: %f", pathPlanningTask->getSideClearanceAtFastSpeed());   //def: 1000
            ArLog::log(ArLog::Normal, "CollRange: %f", pathPlanningTask->getCollisionRange());                       //def: 2000
            ArLog::log(ArLog::Normal, "PlanFreeSpace: %f", pathPlanningTask->getPlanFreeSpace());                    //def: 63x ish
            ArLog::log(ArLog::Normal, "MapRes: %f", pathPlanningTask->getPlanMapResolution());                       //def: 106.25
            ArLog::log(ArLog::Normal, "LogLevel: %s", pathPlanningTask->getLogLevel());                              //def: null
            // ~CONFIG */

            pathPlanningTask->pathPlanToPose(goal, useRotation);
            newGoal = false;

            break;
        case ArPathPlanningTask::PLANNING_PATH: // Planning the inital path
            if (logOutput)
            {
                ArLog::log(ArLog::Normal, "STATE: PLANNING_PATH");
            }

            break;
        case ArPathPlanningTask::MOVING_TO_GOAL: // Moving to the goal
            if (logOutput)
            {
                ArLog::log(ArLog::Normal, "STATE: MOVING_TO_GOAL");
            }

            break;
        case ArPathPlanningTask::REACHED_GOAL: // Reached the goal
            if (logOutput)
            {
                ArLog::log(ArLog::Normal, "STATE: REACHED_GOAL");
            }

            ConfigSingleton::getInstance()->lastPathPlanningSuccess = true;
            reactivateStateMachine();
            break;
        case ArPathPlanningTask::FAILED_PLAN: // Failed to plan a path to goal
            if (logOutput)
            {
                ArLog::log(ArLog::Normal, "STATE: FAILED_PLAN");
            }

            ConfigSingleton::getInstance()->lastPathPlanningSuccess = false;
            reactivateStateMachine();
            break;
        case ArPathPlanningTask::FAILED_MOVE: // Failed to reach goal after plan obtained
            if (logOutput)
            {
                ArLog::log(ArLog::Normal, "STATE: FAILED_MOVE");
            }

            ConfigSingleton::getInstance()->lastPathPlanningSuccess = false;
            reactivateStateMachine();
            break;
        case ArPathPlanningTask::ABORTED_PATHPLAN: // Aborted plan before done
            if (logOutput)
            {
                ArLog::log(ArLog::Normal, "STATE: ABORTED_PATHPLAN");
            }

            ConfigSingleton::getInstance()->lastPathPlanningSuccess = false;
            reactivateStateMachine();
            break;
        case ArPathPlanningTask::INVALID: // Invalid state
            if (logOutput)
            {
                ArLog::log(ArLog::Normal, "STATE: INVALID");
            }

            ConfigSingleton::getInstance()->lastPathPlanningSuccess = false;
            reactivateStateMachine();
            break;
        }
    }
    else if (laser->isTryingToConnect())
    {
        ArLog::log(ArLog::Normal, "Connecting to Laser %s, please wait.",
                   laser->getName());
    }
    else
    {
        laser->asyncConnect();
    }

    laser->unlockDevice();
}

void PathTask::setNewTarget(ArPose &pose, bool useRotation)
{
    goal = pose;
    newGoal = true;
    this->useRotation = useRotation;

    ArLog::log(ArLog::Normal, "Set new target for goToTask");
}

void PathTask::reactivateStateMachine()
{
    ArLog::log(ArLog::Normal, "PathPlanningTask finished. Successfull: %s", ConfigSingleton::getInstance()->lastPathPlanningSuccess ? "true" : "false");

    ConfigSingleton::getInstance()->stateMachine->activate();
}
