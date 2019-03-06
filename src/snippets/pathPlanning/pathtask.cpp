#include "pathtask.h"

#include <ArRobot.h>

PathTask::PathTask(ArRobot *robot, ArPathPlanningTask *pathTask, ArMap *map) :
    robot(robot),
    pathPlanningTask(pathTask),
    goals(map->findMapObjectsOfType("Goal", true)),
    state(ArPathPlanningTask::NOT_INITIALIZED),
    previousState(ArPathPlanningTask::NOT_INITIALIZED)
{
}

void PathTask::invoke()
{
    char text[512];
    state = pathPlanningTask->getState();
    if (state != previousState) {
        pathPlanningTask->getStatusString(text, sizeof(text));
        ArLog::log(ArLog::Normal, "Planning status: %s.", text);
    }

    switch (state) {
    case ArPathPlanningTask::NOT_INITIALIZED: // Task not initialized
        if (goals.size() > 0)  {
            pathPlanningTask->init();
            it = goals.begin();
            pathPlanningTask->pathPlanToPose((*it)->getPose(), false);
        } else {
            ArLog::log(ArLog::Terse, "Warning: Map contains no goals!");
        }
        break;
    case ArPathPlanningTask::PLANNING_PATH: // Planning the inital path
        //TODO: implement state
        break;
    case ArPathPlanningTask::MOVING_TO_GOAL: // Moving to the goal
        //TODO: implement state
        break;
    case ArPathPlanningTask::REACHED_GOAL: // Reached the goal
        ++it;
        if (it == goals.end()) {
            it = goals.begin();
        }
        pathPlanningTask->pathPlanToPose((*it)->getPose(), false);
        break;
    case ArPathPlanningTask::FAILED_PLAN: // Failed to plan a path to goal
        //TODO: implement state
        break;
    case ArPathPlanningTask::FAILED_MOVE: // Failed to reach goal after plan obtained
        //TODO: implement state
        break;
    case ArPathPlanningTask::ABORTED_PATHPLAN: // Aborted plan before done
        //TODO: implement state
        break;
    case ArPathPlanningTask::INVALID: // Invalid state
        //TODO: implement state
        break;
    }
    previousState = state;
}
