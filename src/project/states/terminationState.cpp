#include "terminationState.h"
#include <Aria.h>

void TerminationState::enterState()
{
    ArLog::log(ArLog::Normal, "Entered TerminationState, terminating...");
    myStateMachine->deactivate();

    //TODO deactivate robot
}

ArActionDesired *TerminationState::updateState(const ArActionDesired &currenDesired)
{
    return nullptr;
}

void TerminationState::exitState()
{
}
