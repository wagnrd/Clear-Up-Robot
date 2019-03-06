#ifndef PATHTASK_H
#define PATHTASK_H

#include <ArFunctor.h>
#include <ArPathPlanningTask.h>

class ArRobot;

class PathTask : public ArFunctor
{
public:
    PathTask(ArRobot *robot, ArPathPlanningTask *pathPlanningTask, ArMap *map);
    virtual void invoke();

protected:
    ArRobot *robot;
    ArPathPlanningTask *pathPlanningTask;
    std::list<ArMapObject *> goals;
    std::list<ArMapObject *>::iterator it;
    ArPathPlanningTask::PathPlanningState state;
    ArPathPlanningTask::PathPlanningState previousState;
};

#endif // PATHTASK_H
