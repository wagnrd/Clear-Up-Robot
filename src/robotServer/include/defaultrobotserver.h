#ifndef DEFAULTROBOTSERVER_H
#define DEFAULTROBOTSERVER_H

#include "robotserver.h"

class ArServerHandlerCommands;
class ArServerInfoRobot;
class ArServerInfoSensor;
class ArServerInfoPath;
class ArRobotConnector;
class ArLaserConnector;
class ArArgumentParser;
class ArCompassConnector;
class ArServerBase;
class ArServerSimpleOpener;
class ArServerInfoDrawings;
class ArServerHandlerCommMonitor;
class ArServerHandlerMap;
class ArServerInfoLocalization;
class ArServerHandlerLocalization;
#ifndef WIN32
class ArPTZConnector;
#endif //WIN32

class DefaultRobotServer: public RobotServer
{
public:

    DefaultRobotServer();
    ~DefaultRobotServer();

    virtual void init(int argc, char** argv);
    virtual void run();

protected:
    virtual void logOptions() const;
#ifndef WIN32
    ArPTZConnector *cameraConnector;
#endif //WIN32
    ArCompassConnector *compassConnector;
    ArLaserConnector *laserConnector;
    ArArgumentParser *parser;
    ArRobotConnector *robotConnector;
    ArServerBase *server;
    ArServerInfoDrawings *serverInfoDrawings;
    ArServerSimpleOpener *serverOpener;
    ArServerHandlerCommands *commands;
    ArServerInfoRobot *robotInfo;
    ArServerInfoSensor *sensorInfo;
    ArServerInfoPath *pathInfo;
    ArServerHandlerCommMonitor *handlerCommMonitor;
    ArServerHandlerMap *serverMap;
    ArServerInfoLocalization *serverInfoLocalization;
    ArServerHandlerLocalization *serverLocHandler;
};

#endif // DEFAULTROBOTSERVER_H
