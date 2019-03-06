#pragma once

#include "../stateMachine/state.h"

/*
 * A State used to shut down our state machine
 * 
 * AUTHOR: Jonas Eckstein
 */
class TerminationState : public State
{
  public:
    void enterState();
    ArActionDesired *updateState(const ArActionDesired &currentDesired);
    void exitState();
};