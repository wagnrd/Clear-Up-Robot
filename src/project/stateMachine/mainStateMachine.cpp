#include "mainStateMachine.h"
#include "state.h"
#include "../states/goToToyState.h"
#include "../states/focusOnColorState.h"
#include "../states/takeToyState.h"
#include "../states/terminationState.h"
#include "../states/waitForColorSignalState.h"
#include "../config.h"
#include <memory>
#include <map>
#include <ArLaser.h>
#include "defaultrobotserver.h"
#include "../states/driveToTargetState.h"
#include "../states/goToPlayAreaState.h"
#include "../states/goToChestState.h"
#include "../states/putToyState.h"
#include "../states/searchToyState.h"
#include "../states/goToHomeState.h"
#include "../states/goToChestState.h"

MainStateMachine::MainStateMachine(DefaultRobotServer *server)
    : ArAction("MainStateMachine", "Main state machine which controls the execution of all subprogramms."), initialized(false)
{
    ArLog::log(ArLog::Normal, "before state creation");
    addState(Action::WAITING_FOR_INPUT, std::make_shared<WaitForColorSignalState>());
    addState(Action::FOCUS_ON_TOY, std::make_shared<FocusOnColorState>());
    addState(Action::GO_TO_TOY, std::make_shared<GoToToyState>());
    addState(Action::TAKE_TOY, std::make_shared<TakeToyState>());
    addState(Action::TERMINATE, std::make_shared<TerminationState>());
    addState(Action::GO_TO_COORDINATE, std::make_shared<DriveToTargetState>(server));
    addState(Action::GO_TO_PLAY_AREA, std::make_shared<GoToPlayAreaState>());
    addState(Action::PUT_TOY, std::make_shared<PutToyState>());
    addState(Action::SEARCH_THROUGH_AREA, std::make_shared<SearchToyState>());
    addState(Action::GO_TO_HOME, std::make_shared<GoToHomeState>());
    addState(Action::GO_TO_CHEST, std::make_shared<GoToChestState>());

    currentState = actionStateMap[Action::WAITING_FOR_INPUT];
    ConfigSingleton::getInstance()->laser = server->getLaser();
    ConfigSingleton::getInstance()->sonar = server->getSonar();
    ConfigSingleton::getInstance()->stateMachine = this;
    ConfigSingleton::getInstance()->map = server->getMap();
}

ArActionDesired *MainStateMachine::fire(ArActionDesired currentDesired)
{
    return currentState->updateState(currentDesired);
}

void MainStateMachine::changeState(Action action)
{
    currentState->exitState();
    currentState = actionStateMap.at(action);
    currentState->enterState();
}

void MainStateMachine::addState(Action action, std::shared_ptr<State> state)
{
    if (actionStateMap.count(action)) //0 is interpreted as false, 1 as true
    {
        throw new std::runtime_error("state already exists");
    }
    state->setStateMachine(this);
    actionStateMap[action] = state;
}

void MainStateMachine::activate()
{
    ArAction::activate();

    if (!initialized)
    {
        initialized = true;
        myRobot->getPTZ()->reset();
        ConfigSingleton::getInstance()->camera = myRobot->getPTZ();
        if (!ConfigSingleton::getInstance()->acts.isConnected())
        {
            ConfigSingleton::getInstance()->acts.openPort(myRobot);
        }

        if (!ConfigSingleton::getInstance()->acts.isConnected())
        {
            ArLog::log(ArLog::Normal, "Can not connect to acts");
            currentState = actionStateMap[Action::TERMINATE];
        }

        ConfigSingleton::getInstance()->gripper = new ArGripper(myRobot);
        ConfigSingleton::getInstance()->gripper->gripperStore();

        ConfigSingleton::getInstance()->robot = myRobot;
    }
    ArLog::log(ArLog::Normal, "MainStateMachine activated");

    currentState->enterState();
}

void MainStateMachine::deactivate()
{
    currentState->exitState();
    ArAction::deactivate();
    ArLog::log(ArLog::Normal, "MainStateMachine deactivated");
}
