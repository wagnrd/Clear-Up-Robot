#include "takeToyState.h"
#include "../config.h"
#include "../utils/utils.hpp"

#define USE_GRIPPER

void TakeToyState::enterState()
{
    ArLog::log(ArLog::Normal, "Entering TakeToyState");

    gripper = ConfigSingleton::getInstance()->gripper;
    toyInPlace = false;

    ArLog::log(ArLog::Normal, "gripClose()");
    Utils::gripperClose();
}

ArActionDesired *TakeToyState::updateState(const ArActionDesired &currentDesired)
{
#ifndef USE_GRIPPER
    ArLog::log(ArLog::Normal, "No gripper to operate!");
#endif

#ifdef USE_GRIPPER
    if (toyInPlace && gripper->isLiftMaxed())
    {
        ArLog::log(ArLog::Normal, "Toy in place");
        myStateMachine->changeState(MainStateMachine::Action::GO_TO_CHEST);
    }
    else if (!toyInPlace && gripper->getPaddleState() == 3)
    {
        ArLog::log(ArLog::Normal, "savelyLiftUp()");
        Utils::gripperLiftUp();
        toyInPlace = true;
    }

#endif

    return &myDesired;
}

void TakeToyState::exitState()
{
    ArLog::log(ArLog::Normal, "Exiting TakeToyState");
}
