#pragma once

#include "../stateMachine/state.h"

/*
 * Commands the robot to drive to the goal "home" and to wait for a new color signal
 * 
 * AUTHOR: Jonas Eckstein
 */
class GoToHomeState : public State
{
  public:
    void enterState();
    ArActionDesired *updateState(const ArActionDesired &currentDesired);
    void exitState();
};