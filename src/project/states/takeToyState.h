#pragma once
#include "../stateMachine/state.h"
#include <memory>

/*
 * The robot uses the gripper to pick up the toy and switches to goToChestState
 * 
 * AUTHOR: Denis Wagner
 */

class TakeToyState : public State
{
public:
  virtual void enterState();
  virtual ArActionDesired *updateState(const ArActionDesired &currentDesired);
  virtual void exitState();

private:
  ArGripper *gripper;
  bool toyInPlace;
};