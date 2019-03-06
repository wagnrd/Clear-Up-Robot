#pragma once
#include "../stateMachine/state.h"
#include <ArLaser.h>
#include <ArRobot.h>

/*
 * Searches for a toy in the playare by rotating and looking for large ACTS blobs.
 * It uses the two enclosing walls the detect the end of the playarea
 * 
 * AUTHOR: Jonas Eckstein 
 */
class SearchToyState : public State
{
public:
  virtual void enterState();
  virtual ArActionDesired *updateState(const ArActionDesired &currentDesired);
  virtual void exitState();

private:
  double readingBoxX1;
  double readingBoxX2;
  double readingBoxLeftY1;
  double readingBoxLeftY2;
  bool initialized = false;
  bool goingToStartPos;
  ArLaser *laser;
  ArPose reading;

  double distanceTreshold = 800;
  double rotationSpeed = 20;
  double cameraTilt = -20;
  double minBlobSize = 270; //intentionally a little bit bigger than the minBlobSize of chaseColorState
};