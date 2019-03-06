#pragma once

#include "../stateMachine/state.h"
#include <memory>

/*
 * The robot will drive towards the toy box (chest) back wall and places it as close as possible to the wall
 * or other toys to reduce space usage. It will then drive backwards and tries to detect obstacles behind
 * and stops if detected
 * 
 * AUTHOR: Denis Wagner
 */

class PutToyState : public State
{
public:
  virtual void enterState();
  virtual ArActionDesired *updateState(const ArActionDesired &currentDesired);
  virtual void exitState();

private:
  enum class SubState
  {
    INITIAL_STATE,
    TOY_ON_FLOOR,
    GRIPPER_OPEN,
    ROBOT_IN_DEPLOY_POS,
    ROBOT_IN_SAVE_POS
  };

  ArGripper *gripper;
  bool toyOnFloor;
  bool gripperUp;
  SubState currentSubState;
  int objectsInBox = 0;
  int waitingCounter;
  int distanceToWall = 100;
  int distanceBetweenToys = 120;
};