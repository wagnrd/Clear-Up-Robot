#pragma once

#include "../stateMachine/state.h"

/*
 * Commands the robot to drive to the goal "playarea" and to start searching for toys
 * 
 * AUTHOR: Jonas Eckstein
 */
class GoToPlayAreaState : public State
{
public:
  void enterState();
  ArActionDesired *updateState(const ArActionDesired &currentDesired);
  void exitState();
};