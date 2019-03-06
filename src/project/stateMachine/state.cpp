#include "state.h"

State::~State() { }

void State::setStateMachine(MainStateMachine* stateMachine)
{
    myStateMachine = stateMachine;
}

ArRobot* State::getRobot()
{
    return myStateMachine->myRobot;
}
