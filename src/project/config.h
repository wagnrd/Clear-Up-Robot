#pragma once
#include <ArACTS.h>
#include "stateMachine/mainStateMachine.h"

class ArGripper;
class MainStateMachine;

/*
 * Contains "global variables".
 * We could have used "noramal" global variables instead
 * 
 * 
 * AUTHOR:
 * Singleton implementation: Jonas Eckstein
 * Addition of new variables also done by Denis Wagner
 * 
 */
class ConfigSingleton
{
public:
  ~ConfigSingleton();
  ConfigSingleton(const ConfigSingleton &other) = delete;
  ConfigSingleton(ConfigSingleton &&other) = delete;

  static ConfigSingleton *getInstance();

  /*
   * for manuel config
   */

  const int cameraWith = 720;
  const int cameraHeight = 480;

  const double focalLength = 72;

  /*
   * NOT for manual config
   * 
   * these are kind of global variables
   */

  ArMap *map;
  int targetChannel = -1;
  ArACTS_1_2 acts;
  ArGripper *gripper;
  ArPTZ *camera;
  ArRobot *robot;
  bool isGripperUp = true;
  bool isGripperOpen = false;
  ArLaser *laser;
  ArSonarDevice *sonar;
  MainStateMachine *stateMachine;

  //target and return state for path planing
  MainStateMachine::Action returnState;
  ArPose targetPose;
  bool useRotation;
  bool lastPathPlanningSuccess = true;

private:
  ConfigSingleton() = default;
  static ConfigSingleton *instance; //does not need to be deleted, is needed until programm termination
};