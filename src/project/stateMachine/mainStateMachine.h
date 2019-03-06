#pragma once
#include <ArAction.h>
#include <Aria.h>
#include <memory>
#include <vector>

class ArLaser;
class DefaultRobotServer;
class State;
/*
 * The main state machine for or robot logic
 * 
 * AUTHOR: Denis Wagner
 * some adjustments like bugfixes, adding new states and initializing more variables where also done by Jonas Eckstein
 */
class MainStateMachine : public ArAction
{
  friend class State;

public:
  enum class Action
  {
    WAITING_FOR_INPUT,
    GO_TO_PLAY_AREA,
    FOCUS_ON_TOY,
    GO_TO_TOY,
    GO_TO_CHEST,
    GO_TO_HOME,
    TAKE_TOY,
    PUT_TOY,
    SEARCH_THROUGH_AREA,
    GO_TO_COORDINATE,
    TERMINATE
  };

  MainStateMachine(DefaultRobotServer *server);

  ArActionDesired *fire(ArActionDesired currentDesired);
  void changeState(Action action);
  void addState(Action action, std::shared_ptr<State>);
  virtual void activate();
  virtual void deactivate();

private:
  std::shared_ptr<State> currentState;
  std::map<Action, std::shared_ptr<State>> actionStateMap;
  bool initialized;
};