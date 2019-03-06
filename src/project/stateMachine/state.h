#pragma once

#include <ArRobot.h>
#include <Aria.h>
#include "mainStateMachine.h"

/*
 * base class for all of our states
 * 
 * AUTHOR: Denis Wagner
 */

class State
{
protected:
  MainStateMachine *myStateMachine;
  ArRobot *getRobot();
  ArActionDesired myDesired;

public:
  virtual ~State();

  virtual void enterState() = 0;
  virtual ArActionDesired *updateState(const ArActionDesired &) = 0;
  virtual void exitState() = 0;
  void setStateMachine(MainStateMachine *stateMachine);
};
