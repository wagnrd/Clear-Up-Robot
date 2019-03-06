#include "goToHomeState.h"
#include <ArMap.h>
#include "../config.h"
#include <Aria.h>

void GoToHomeState::enterState()
{
    ArLog::log(ArLog::Normal, "Entering GoToHomeState");

    ConfigSingleton::getInstance()->targetPose = ConfigSingleton::getInstance()->map->findFirstMapObject("home", "Goal", true)->getPose();
    ArLog::log(ArLog::Normal, "New Target: %f, %f", ConfigSingleton::getInstance()->targetPose.getX(), ConfigSingleton::getInstance()->targetPose.getY());
    ConfigSingleton::getInstance()->returnState = MainStateMachine::Action::WAITING_FOR_INPUT;
    ConfigSingleton::getInstance()->useRotation = true;
    myStateMachine->changeState(MainStateMachine::Action::GO_TO_COORDINATE);
}
ArActionDesired *GoToHomeState::updateState(const ArActionDesired &currentDesired)
{
}
void GoToHomeState::exitState()
{
    ArLog::log(ArLog::Normal, "Exiting GoToHomeState");
}