#include "goToPlayAreaState.h"
#include <ArMap.h>
#include "../config.h"
#include <Aria.h>

void GoToPlayAreaState::enterState()
{
    ArLog::log(ArLog::Normal, "Entering GoToPlayAreaState");

    ConfigSingleton::getInstance()->targetPose = ConfigSingleton::getInstance()->map->findFirstMapObject("playarea", "Goal", true)->getPose();
    ArLog::log(ArLog::Normal, "New Target: %f, %f", ConfigSingleton::getInstance()->targetPose.getX(), ConfigSingleton::getInstance()->targetPose.getY());
    ConfigSingleton::getInstance()->returnState = MainStateMachine::Action::SEARCH_THROUGH_AREA;
    ConfigSingleton::getInstance()->useRotation = true;
    myStateMachine->changeState(MainStateMachine::Action::GO_TO_COORDINATE);
}
ArActionDesired *GoToPlayAreaState::updateState(const ArActionDesired &currentDesired)
{
}
void GoToPlayAreaState::exitState()
{
    ArLog::log(ArLog::Normal, "Exiting GoToPlayAreaState");
}