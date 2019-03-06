#pragma once

#include "../stateMachine/state.h"
#include "../utils/goToTask.h"

class DefaultRobotServer;

/*
 * Drives the robot to the goal specified in ConfigSingleton.
 * 
 * AUTHOR: Jonas Eckstein
 */
class DriveToTargetState : public State
{
public:
  DriveToTargetState(DefaultRobotServer *server);
  void enterState();
  ArActionDesired *updateState(const ArActionDesired &currentDesired);
  void exitState();

private:
  PathTask pathTask;

  bool pathPlanningIsRunning;
};