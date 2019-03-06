#pragma once
#include "../stateMachine/state.h"

/*
 * Performs some startup tasks, like turning on our laser.
 * After evrything is ready he waits for some large blob on some ACTS channel 
 * and sets this channel as target channel in config
 * 
 * AUTHOR: Jonas Eckstein
 */

class WaitForColorSignalState : public State
{
public:
  void enterState();
  ArActionDesired *updateState(const ArActionDesired &currentDesired);
  void exitState();

private:
  ArACTS_1_2 *acts;
  ArACTSBlob tmpBlob;
  int largestBlobArea;
  int blobsCount;

  //to config:
  //minimum size of a blob to be tracked
  const int minBlobArea = 1000;
  //the state to go in after start signal is detected
  MainStateMachine::Action nextState = MainStateMachine::Action::GO_TO_PLAY_AREA;
};