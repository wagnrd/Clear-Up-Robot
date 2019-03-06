#include "robotserver.h"

#include <Aria.h>
#include <Arnl.h>

RobotServer::RobotServer() :
    gripper(0),
    gyro(0),
    keyHandler(0),
    laser(0),
    localizationTask(0),
    map(0),
    pathTask(0),
    robot(0),
    sonarDev(0)
{
}

RobotServer::~RobotServer()
{
    delete gripper;
    delete gyro;
    delete keyHandler;
    delete laser;
    delete localizationTask;
    delete map;
    delete pathTask;
    delete robot;
    delete sonarDev;
}

bool RobotServer::addAction(ArAction *action, int priority)
{
    if(robot && action) {
        return robot->addAction(action, priority);
    }
    return false;
}

bool RobotServer::addAction(ArAction &action, int priority)
{
    return addAction(&action, priority);
}

ArRobot *RobotServer::getRobot()
{
    if (!robot) ArLog::log(ArLog::Normal, "Warning: robot requested, but it is NULL!");
    return robot;
}

ArSonarDevice *RobotServer::getSonar()
{
    if (!sonarDev) ArLog::log(ArLog::Normal, "Warning: sonar requested, but it is NULL!");
    return sonarDev;
}

ArLaser *RobotServer::getLaser()
{
    if (!laser) ArLog::log(ArLog::Normal, "Warning: laser requested, but it is NULL!");
    return laser;
}

ArGripper *RobotServer::getGripper()
{
    if (!gripper) ArLog::log(ArLog::Normal, "Warning: gripper requested, but it is NULL!");
    return gripper;
}

ArMap *RobotServer::getMap()
{
    if (!map) ArLog::log(ArLog::Normal, "Warning: map requested, but it is NULL!");
    return map;
}

ArPathPlanningTask *RobotServer::getPathTask()
{
    if (!pathTask) ArLog::log(ArLog::Normal, "Warning: pathTask requested, but it is NULL!");
    return pathTask;
}

ArAnalogGyro *RobotServer::getGyro()
{
    if (!gyro) ArLog::log(ArLog::Normal, "Warning: gyro requested, but it is NULL!");
    return gyro;
}

ArKeyHandler *RobotServer::getKeyHandler()
{
    if (!keyHandler) ArLog::log(ArLog::Normal, "Warning: keyHandler requested, but it is NULL!");
    return keyHandler;
}

ArBaseLocalizationTask *RobotServer::getLocalizationTask()
{
    if (!localizationTask) ArLog::log(ArLog::Normal, "Warning: localizationTask requested, but it is NULL!");
    return localizationTask;
}

ArPTZ *RobotServer::getCamera()
{
    ArPTZ *camera = 0;
    if (robot) camera = robot->getPTZ();
    if (!camera) ArLog::log(ArLog::Normal, "Warning: camera requested, but it is NULL!");
    return camera;
}
