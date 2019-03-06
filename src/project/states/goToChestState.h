#pragma once

#include "../stateMachine/state.h"

/*
 * State that lets the robot drive to the chest and switches to putToyState
 * 
 * AUTHOR: Denis Wagner 
 */

class GoToChestState : public State
{
public:
  void enterState();
  ArActionDesired *updateState(const ArActionDesired &currentDesired);
  void exitState();
};