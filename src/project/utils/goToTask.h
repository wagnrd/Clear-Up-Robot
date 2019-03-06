#pragma once

#include <ArFunctor.h>
#include <ArPathPlanningTask.h>
#include <ArAction.h>

class ArRobot;
class DefaultRobotServer;

/*
 * This class is partially based on the snippet "pathPlanning"
 * Drives the robot to a target using the ArPathPlanningTask
 * 
 * AUTHOR: Jonas Eckstein
 */

class PathTask : public ArFunctor
{
public:
  PathTask(DefaultRobotServer *server);
  virtual void invoke();
  void setNewTarget(ArPose &pose, bool useRotation);
  auto getPathPlanAction()
  {
    return pathPlanningTask->getPathPlanAction();
  }

protected:
  ArRobot *robot;
  ArPathPlanningTask *pathPlanningTask;
  ArPose goal;
  ArPathPlanningTask::PathPlanningState state;
  ArPathPlanningTask::PathPlanningState previousState;
  ArLaser *laser;

private:
  void reactivateStateMachine();
  bool newGoal;
  bool useRotation;
};
