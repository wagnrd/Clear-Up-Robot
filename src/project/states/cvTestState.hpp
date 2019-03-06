#pragma once

#include "../stateMachine/state.h"
#include "cvBallRecognition.hpp"

/*
 * Test state for cvBallRecognition
 * 
 * AUTHOR: Denis Wagner
 */

class CVTestState : public State
{
public:
  void enterState();
  ArActionDesired *updateState(const ArActionDesired &currentDesired);
  void exitState();

private:
  CVBallRecognition cvBallRectognition;
};