#ifndef ROBOTSERVER_H
#define ROBOTSERVER_H

// Forward declarations
class ArAction;
class ArRobot;
class ArSonarDevice;
class ArLaser;
class ArGripper;
class ArPTZ;
class ArAnalogGyro;
class ArKeyHandler;
class ArMap;
class ArPathPlanningTask;
class ArBaseLocalizationTask;

class RobotServer
{
public:
    RobotServer();
    ~RobotServer();

    virtual void init(int argc, char** argv) = 0;
    virtual void run() = 0;

    virtual bool addAction(ArAction *action, int priority=50);
    virtual bool addAction(ArAction &action, int priority=50);

    virtual ArRobot *getRobot();
    virtual ArSonarDevice *getSonar();
    virtual ArLaser *getLaser();
    virtual ArGripper *getGripper();
    virtual ArMap *getMap();
    virtual ArPathPlanningTask *getPathTask();
    virtual ArAnalogGyro *getGyro();
    virtual ArKeyHandler *getKeyHandler();
    virtual ArBaseLocalizationTask *getLocalizationTask();
    virtual ArPTZ *getCamera();

protected:
    ArGripper *gripper;
    ArAnalogGyro *gyro;
    ArKeyHandler *keyHandler;
    ArLaser *laser;
    ArBaseLocalizationTask *localizationTask;
    ArMap *map;
    ArPathPlanningTask *pathTask;
    ArRobot *robot;
    ArSonarDevice *sonarDev;
};

#endif // ROBOTSERVER_H
