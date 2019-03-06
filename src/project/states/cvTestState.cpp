#include "cvTestState.hpp"

void CVTestState::enterState()
{
    ArLog::log(ArLog::Normal, "Entering CVTestState");
    //cvBallRectognition.initOpenCV();
}

ArActionDesired *CVTestState::updateState(const ArActionDesired &currenDesired)
{
    //ArLog::log(ArLog::Normal, "Color is beeing tracked");
    myDesired.setVel(-50);
    return &myDesired;
}

void CVTestState::exitState()
{
    ArLog::log(ArLog::Normal, "Exiting CVTestState");
}