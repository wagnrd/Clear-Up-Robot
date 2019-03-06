#include "defaultrobotserver.h"

#include <Aria.h>
#include <ArSystemStatus.h>
#include <Arnl.h>
#include <string>

DefaultRobotServer::DefaultRobotServer() :
#ifndef WIN32
    cameraConnector(0),
#endif //WIN32
    compassConnector(0),
    laserConnector(0),
    parser(0),
    robotConnector(0),
    server(0),
    serverInfoDrawings(0),
    serverOpener(0),
    commands(0),
    robotInfo(0),
    sensorInfo(0),
    pathInfo(0),
    handlerCommMonitor(0),
    serverMap(0),
    serverInfoLocalization(0),
    serverLocHandler(0)
{
}

DefaultRobotServer::~DefaultRobotServer()
{
#ifndef WIN32
    delete cameraConnector;
#endif //WIN32
    delete compassConnector;
    delete laserConnector;
    delete parser;
    delete robotConnector;
    delete server;
    delete serverInfoDrawings;
    delete serverOpener;
    delete commands;
    delete robotInfo;
    delete sensorInfo;
    delete pathInfo;
    delete handlerCommMonitor;
    delete serverMap;
    delete serverInfoLocalization;
    delete serverLocHandler;
}

void DefaultRobotServer::init(int argc, char** argv)
{
    // Initialize some global data
    Aria::init();

    Arnl::init();

    // If you want ArLog to print "Verbose" level messages uncomment this:
    //ArLog::init(ArLog::StdOut, ArLog::Verbose);

    map = new ArMap();

    // Parse custom arguments
    bool useLaser = true;
    bool useGripper = true;
    bool *argsConsumed = new bool[argc];
    int ariaArgc = argc;
    for (int i = 1; i < argc; ++i) {
        argsConsumed[i] = false;
        ArLog::log(ArLog::Normal, argv[i]);
        if (std::string("-map").compare(argv[i]) == 0 && i+1 < argc) {
            argsConsumed[i] = true;
            argsConsumed[++i] = true;
            ariaArgc -= 2;
            char* mapFileName = argv[i];
            if(!map->readFile(mapFileName)) {
                ArLog::log(ArLog::Terse, "Error: Can't load map file: %s", mapFileName);
                Aria::exit(-1);
                return;
            }
        } else if (std::string("-noGripper").compare(argv[i]) == 0) {
            ArLog::log(ArLog::Normal, "RobotServer: Running in Simulation. Disabling gripper!");
            useGripper = false;
            argsConsumed[i] = true;
            --ariaArgc;
        } else if (std::string("-noLaser").compare(argv[i]) == 0) {
            ArLog::log(ArLog::Normal, "RobotServer: Disable laser!");
            useLaser = false;
            argsConsumed[i] = true;
            --ariaArgc;
        }
    }

    // Remove consumed arguments
    char** ariaArgv = new char*[argc-ariaArgc];
    ariaArgv[0] = argv[0];
    for (int i = 1, j = 1; i < argc; ++i) {
        if (!argsConsumed[i]) {
            ariaArgv[j++] = argv[i];
        }
    }

    delete[] argsConsumed;

    parser = new ArArgumentParser(&ariaArgc, ariaArgv);

    robot = new ArRobot();
    robotConnector = new ArRobotConnector(parser, robot);

    server = new ArServerBase();
    serverOpener = new ArServerSimpleOpener(parser);
#ifndef WIN32
    cameraConnector = new ArPTZConnector(parser, robot);
#endif //WIN32

    // To get CPU usage and wireless information from Linux
    ArSystemStatus::runRefreshThread();

    // Load some default values for command line arguments from /etc/Aria.args
    // (Linux) or the ARIAARGS environment variable.
    parser->loadDefaultArguments();

    if (!Aria::parseArgs() || !parser->checkHelpAndWarnUnparsed())
    {
        logOptions();
        Aria::exit(1);
        return;
    }

    // If the robot has an Analog Gyro, this object will activate it, and
    // if the robot does not automatically use the gyro to correct heading,
    // this object reads data from it and corrects the pose in ArRobot
    gyro = new ArAnalogGyro(robot);

    // add our logging to the config
    ArLog::addToConfig(Aria::getConfig());

    // Connect to the robot, get some initial data from it such as type and name,
    // and then load parameter files for this robot.
    if (!robotConnector->connectRobot())
    {
        // Error connecting:
        // if the user gave the -help argumentp, then just print out what happened,
        // and continue so options can be displayed later.
        if (!parser->checkHelpAndWarnUnparsed())
        {
            ArLog::log(ArLog::Terse, "Could not connect to robot, will not have parameter file so options displayed later may not include everything");
        }
        // otherwise abort
        else
        {
            ArLog::log(ArLog::Terse, "Error, could not connect to robot.");
            logOptions();
            Aria::exit(1);
            return;
        }
    }

    // Set up a class that'll put the movement parameters into the config stuff
    ArRobotConfig robotConfig(robot);
    robotConfig.addAnalogGyro(gyro);

    if(!robot->isConnected())
    {
        ArLog::log(ArLog::Terse, "Internal error: robot connector succeeded but ArRobot::isConnected() is false!");
    }

    laserConnector = new ArLaserConnector(parser, robot, robotConnector);

    compassConnector = new ArCompassConnector(parser);

    sonarDev = new ArSonarDevice();

    keyHandler = new ArKeyHandler();
    Aria::setKeyHandler(keyHandler);

    robot->attachKeyHandler(keyHandler);
    ArLog::log(ArLog::Normal, "You may press escape to exit.");

    robot->addRangeDevice(sonarDev);

    robot->runAsync(true);
#ifndef WIN32
    cameraConnector->connect();

    ArUtil::sleep(500);

    for(int i = 0; i < cameraConnector->getNumPTZs(); ++i) {
        ArPTZ *camera = cameraConnector->getPTZ(i);
        robot->setPTZ(camera);
        break;
    }
#else
    ArUtil::sleep(500);
#endif
    ArTCM2 *compass = compassConnector->create(robot);
        if(compass && !compass->blockingConnect()) {
        compass = NULL;
    }

    if (useLaser) {
        if(!laserConnector->connectLasers(false, false, true))
        {
            ArLog::log(ArLog::Terse, "Could not connect to configured lasers. Exiting.");
            Aria::exit(3);
            return;
        }

        // Allow some time to read laser data
        ArUtil::sleep(500);

        for (int i = 1; i <= 10; i++)
        {
            if ((laser = robot->findLaser(i)) != NULL)
            {

                ArLog::log(ArLog::Normal, "Laser %d: %s found!", i, laser->getName());
                break;
            }
        }

        pathTask = new ArPathPlanningTask(robot, laser, sonarDev, map);
    } else {
        pathTask = new ArPathPlanningTask(robot, sonarDev, map);
    }

    if (!serverOpener->open(server)) {
        ArLog::log(ArLog::Terse, "Warning: Could not open server! MobileEyes and other clients will not work.");
    } else {
        commands = new ArServerHandlerCommands(server);
        robotInfo = new ArServerInfoRobot(server, robot);
        sensorInfo = new ArServerInfoSensor(server, robot);
        pathInfo = new ArServerInfoPath(server, robot, pathTask);
        pathInfo->addControlCommands(commands);
    }

    if (useGripper && robot->getOrigRobotConfig()->getHasGripper()) {
        gripper = new ArGripper(robot);
    }

    ArAction::setDefaultActivationState(false);

    serverInfoDrawings = new ArServerInfoDrawings(server);
    serverInfoDrawings->addRobotsRangeDevices(robot);

    handlerCommMonitor = new ArServerHandlerCommMonitor(server);
    serverMap = new ArServerHandlerMap(server, map);

#ifdef ARNL
    if (useLaser) {
        localizationTask = new ArLocalizationTask(robot, laser, map);
    }
#endif // ARNL
#if SONARNL
    if (!localizationTask) {
        localizationTask = new ArSonarLocalizationTask(robot, sonarDev, map);
    }
#endif // SONARNL
    if (localizationTask) {
        serverInfoLocalization = new ArServerInfoLocalization(server, robot, localizationTask);
        serverLocHandler = new ArServerHandlerLocalization(server, robot, localizationTask);
    }

    server->runAsync();
}

void DefaultRobotServer::run()
{

    robot->clearDirectMotion();
    robot->enableMotors();
    robot->waitForRunExit();
    Aria::exit(0);
}

void DefaultRobotServer::logOptions() const
{
    Aria::logOptions();
    ArLog::log(ArLog::Terse, "RobotServer options:");
    ArLog::log(ArLog::Terse, "-map <mapFileName>");
    ArLog::log(ArLog::Terse, "-noLaser");
    ArLog::log(ArLog::Terse, "-noGripper");
}
