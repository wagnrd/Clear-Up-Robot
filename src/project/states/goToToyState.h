#pragma once
#include "../stateMachine/state.h"
#include "../utils/pid.hpp"
#include <memory>

/*
 * The robot will drive towards the focused toy with varying speed depending of the distance to the toy
 * and stops if it is to close to an obstacle or detects a wrong toy in its drive way
 * 
 * AUTHOR: Denis Wagner
 * Concept for avoidence of toys with different colors as well as help with implementing it: Jonas Eckstein
 */

class GoToToyState : public State
{
public:
  void enterState();
  ArActionDesired *updateState(const ArActionDesired &currentDesired);
  void exitState();

private:
  ArACTS_1_2 *acts;
  ArGripper *gripper;
  int blobNotFoundCounter;
  double distanceMedianMM;
  Utils::PidController<double> xAxisPid = Utils::PidController<double>(0.08, 0, 0.09); //(0.3, 0.05, 0.1);
  Utils::PidController<double> yAxisPid = Utils::PidController<double>(0.1, 0, 0.02);  //(0.1, 0.06, 0.02);
  ArPTZ *camera;
  const int minCameraAreaX = 80;
  const int maxCameraAreaX = 640;
  const int minCameraAreaY = 330;
  const int maxCameraAreaY = 480;
  const int minBlobArea = 250;
  const double maxSideDistance = 10;
  const double maxFrontDistance = 115;
  int frameWithoutBlobCounter;
  bool tooCloseLeft;
  bool tooCloseRight;
  int tooCloseCounter;
  int cantAccessToyCounter;
  bool ignoreWrongToy;
  int wrongToyBehindCounter;
  bool toyBrokeOuterBeam;
  int toyInGripperCounter;
};