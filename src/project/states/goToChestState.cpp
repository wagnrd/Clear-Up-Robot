#include "goToChestState.h"
#include <ArMap.h>
#include "../config.h"
#include <Aria.h>

void GoToChestState::enterState()
{
    ArLog::log(ArLog::Normal, "Entering GoToPlayChestState");
    ConfigSingleton *config = ConfigSingleton::getInstance();

    config->targetPose = config->map->findFirstMapObject("chest", "Goal", true)->getPose();
    ArLog::log(ArLog::Normal, "New Target: %f, %f", config->targetPose.getX(), config->targetPose.getY());
    config->returnState = MainStateMachine::Action::PUT_TOY;
    config->useRotation = true;
    myStateMachine->changeState(MainStateMachine::Action::GO_TO_COORDINATE);
}

ArActionDesired *GoToChestState::updateState(const ArActionDesired &currentDesired)
{
    // pass
}

void GoToChestState::exitState()
{
    ArLog::log(ArLog::Normal, "Exiting GoToChestState");
}