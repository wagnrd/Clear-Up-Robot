/*
Adept MobileRobots Advanced Robotics Navigation and Localization (ARNL)
Version 1.8.1

Copyright (C) 2004, 2005 ActivMedia Robotics LLC
Copyright (C) 2006, 2007, 2008, 2009 MobileRobots Inc.
Copyright (C) 2010, 2011, 2012, 2013 Adept Technology, Inc.

All Rights Reserved.

Adept MobileRobots does not make any representations about the
suitability of this software for any purpose.  It is provided "as is"
without express or implied warranty.

The license for this software is distributed as LICENSE.txt in the top
level directory.

robots@mobilerobots.com
Adept MobileRobots
10 Columbia Drive
Amherst, NH 03031
+1-603-881-7960

*/
#ifdef _GPP_OMIT_
          /* proprietaryJustLocalizationTempl.cpp 
           * Combined example of just localization, used as template to generate users' examples.
           *
           * This program can be used as the input to the a preprocessor to
           * generate versions for ARNL, SONARNL, MOGS
           * but it is also by itself a usable program, which uses MCL if ARNL
           * is defined, sonar localization if SONARNL is defined but ARNL is not defined,
           * and GPS localization if either is defined and MOGS is enabled.
           *
           * TODO:
           *  Only include localization manager object if ARNL_MULTILOC is
           *  defined (when more than one localization technique is selected.)
           *
           */
#endif


#if defined(ARNL)
/** @example arnlJustLocalization.cpp Example which does localization only,
  but provides servers for remote operation with MobileEyes. */
#elif defined(SONARNL)
/** @example sonarnlJustLocalization.cpp Example which does localization only,
  but provides servers for remote operation with MobileEyes. */
#elif defined (MOGS)
/** @example mogsJustLocalization.cpp Example which does localization only,
  but provides servers for remote operation with MobileEyes. */
#endif
#ifdef ARNL
#warning Arnl selected. Enabling laser localization, docking, multirobot, mapping.  Skipping SonArnl.
#define ARNL_LASERLOC
#elif defined(SONARNL)
#warning Sonarnl selected.  Enabling sonar localization.
#define ARNL_SONARLOC
#endif
#ifdef MOGS
#warning Mogs selected. Enabling GPS localization.
#define ARNL_GPSLOC
#if defined(ARNL) || defined(SONARNL)
#define ARNL_MULTILOC
#endif
#endif
#if defined(MOGS) || defined(ARNL)
#define ARNL_SICK
#endif
#if !defined(MOGS)  && !defined(ARNL) && !defined(SONARNL) 
#error Must have at least one of MOGS, ARNL, SONARNL defined!
#endif
 
/**
This example server only does localization and not path planning.  
To test this you can start up MobileEyes and connect to the robot 
then just drive the robot around with teleoperation commands.

Use this example as a starting point for adding your own control behavior,
but using ARNL for localization.

Note, to make your own custom planning/navigation information and controls
available in MobileEyes you can look at ArServerClasses.cpp and make your own class
that adds the same server commands as ArServerInfoPath and
ArServerModeGoto.

*/

#include "Aria.h"
#include "ArNetworking.h"
#include "Arnl.h"

#ifdef ARNL_LASERLOC
#include "ArLocalizationTask.h"
#endif

#ifdef ARNL_DOCKING
#include "ArDocking.h"
#endif

#ifdef ARNL_SONARLOC
#include "ArSonarLocalizationTask.h"
#endif

int main(int argc, char *argv[])
{
  Aria::init();
  Arnl::init();

  ArRobot robot;
  ArServerBase server;
  ArArgumentParser parser(&argc, argv);
  parser.loadDefaultArguments();
  ArSimpleConnector simpleConnector(&parser);
  ArServerSimpleOpener simpleOpener (&parser);
#ifdef ARNL_GPSLOC
  ArGPSConnector gpsConnector(&parser);
#endif
  ArAnalogGyro gyro (&robot);

  parser.loadDefaultArguments();
  if (!Aria::parseArgs () || !parser.checkHelpAndWarnUnparsed())
  {
    Aria::logOptions ();
    Aria::exit (1);
  }

#ifdef ARNL_SICK
  ArSick sick;
  robot.addRangeDevice (&sick);
#endif 

  ArSonarDevice sonarDev;
  robot.addRangeDevice (&sonarDev);

  ArBumpers bumpers;
  robot.addRangeDevice (&bumpers);

  // Use the "examples" directory as a place to keep map files
  char fileDir[1024];
  ArUtil::addDirectories (fileDir, sizeof (fileDir), Aria::getDirectory (),
			"examples");

  // Set up the map, this will look for files in the examples
  // directory (unless the file name starts with a / \ or .
  // You can take out the argument to look in the current directory
  ArMap arMap (fileDir);
  // set it up to ignore empty file names (otherwise the parseFile
  // on the config will fail)
  arMap.setIgnoreEmptyFileName (true);


  ArLocalizationManager locManager(&robot, &arMap);
#ifdef ARNL_LASERLOC
  ArLog::log(ArLog::Normal, "Creating laser localization task");
  ArLocalizationTask locTask (&robot, &sick, &arMap);
  locManager.addLocalizationTask(&locTask);
#endif

#ifdef ARNL_SONARLOC
  ArLog::log(ArLog::Normal, "Creating sonar localization task");
  ArSonarLocalizationTask locTask (&robot, &sonarDev, &arMap);
  locManager.addLocalizationTask(&locTask);
#endif 

#ifdef ARNL_GPSLOC
  ArLog::log(ArLog::Normal, "Connecting to GPS...");
  ArGPS *gps = gpsConnector.createGPS(&robot);
  if(!gps || !gps->connect())
  {
    ArLog::log(ArLog::Terse, "Error connecting to GPS device."
      "Try -gpsType, -gpsPort, and/or -gpsBaud command-line arguments."
      "Use -help for help. Exiting.");
    Aria::exit(5);
  }
  ArLog::log(ArLog::Normal, "Creating GPS localization task");
  ArGPSLocalizationTask gpsLocTask(&robot, gps, &arMap);
#endif

  // Add log controls to configuration
  ArLog::addToConfig (Aria::getConfig ());

  // Open the server port
  if (!simpleOpener.open (&server, fileDir, 240))
  {
    ArLog::log (ArLog::Normal, "Could not open server port");
    exit (2);
  }

  // Connect to the robot
  if (!simpleConnector.connectRobot (&robot))
  {
    ArLog::log (ArLog::Normal, "Could not connect to robot... exiting");
    Aria::exit (3);
  }

  robot.enableMotors ();
  robot.clearDirectMotion ();

  // reset the simulator to its start position
  robot.com(ArCommands::SIM_RESET);

#ifdef ARNL_SICK
  simpleConnector.setupLaser (&sick);
#endif 

  // run robot task cycle
  robot.runAsync (true);

#ifdef ARNL_SICK
  // Run SICK thread and try to connect
  sick.runAsync ();
  if (!sick.blockingConnect ())
  {
    ArLog::log (ArLog::Normal, "Couldn't connect to laser, exiting");
    Aria::exit (4);
  }
#endif 

  ArUtil::sleep (300);

  // Forbidden regions from the map.
  ArForbiddenRangeDevice forbidden (&arMap);
  robot.addRangeDevice (&forbidden);

  // Objects that provide network services:
  ArServerInfoRobot serverInfoRobot (&server, &robot);
  ArServerInfoSensor serverInfoSensor (&server, &robot);
  ArServerInfoLocalization serverInfoLocalization (&server, &robot, &locManager);
  ArServerHandlerLocalization serverLocHandler (&server, &robot, &locManager);
  ArServerHandlerMap serverMap (&server, &arMap);

  // Drawing services in the map display:
  ArServerInfoDrawings drawings (&server);
  drawings.addRobotsRangeDevices (&robot);

  // Misc. "custom " commands:
  ArServerHandlerCommands commands (&server);
  ArServerSimpleComUC uCCommands (&commands, &robot);
  ArServerSimpleComMovementLogging loggingCommands (&commands, &robot);
  ArServerSimpleComGyro gyroCommands (&commands, &robot, &gyro);
  ArServerSimpleComLogRobotConfig configCommands (&commands, &robot);

  // Set up the possible modes for remote control from a client such as
  // MobileEyes:

  // To stop and remain stopped:
  ArServerModeStop modeStop (&server, &robot);
  modeStop.addAsDefaultMode ();

#ifndef ARNL_SONARLOC
  // Disable the sonar automatically if stopped and enable when we
  // move
  ArSonarAutoDisabler sonarAutoDisabler (&robot);
#endif

  // Teleoperate by keyboard, joystick, etc:
  ArServerModeRatioDrive modeRatioDrive (&server, &robot);

  // Teloperation mode's configuration and special commands:
  modeRatioDrive.addControlCommands (&commands);
  modeRatioDrive.addToConfig (Aria::getConfig (), "Teleop settings");

  // Wander mode:
  ArServerModeWander
  modeWander (&server, &robot);

  // Prevent driving if localization is lost:
  ArActionLost actionLostRatioDrive (&locManager, NULL, &modeRatioDrive);
  modeRatioDrive.getActionGroup ()->addAction (&actionLostRatioDrive, 110);

  // Prevent wandering if lost:
  ArActionLost
  actionLostWander (&locManager, NULL, &modeWander);
  modeWander.getActionGroup ()->addAction (&actionLostWander, 110);

  // This provides a small table of interesting information for the client
  // to display to the operator:
     ArServerInfoStrings
     stringInfo (&server);
  Aria::getInfoGroup ()->addAddStringCallback (stringInfo.
					     getAddStringFunctor ());

  // Display localization score and more
#ifdef ARNL_SONARLOC
  Aria::getInfoGroup ()->addStringDouble ("Localization Score", 8,
	new ArRetFunctorC < double, ArSonarLocalizationTask > (&locTask,
							   &ArSonarLocalizationTask::
							   getLocalizationScore),
					  		"%.03f");
#elif defined(ARNL_LASERLOC)
  Aria::getInfoGroup ()->addStringDouble ("Localization Score", 8,
					new ArRetFunctorC < double,
					ArLocalizationTask > (&locTask,
							      &ArLocalizationTask::
							      getLocalizationScore),
					"%.03f");
#endif
#ifdef ARNL_SICK
  Aria::getInfoGroup ()->addStringInt ("Laser Packet Count", 10,
				     new ArRetFunctorC < int, ArSick > (&sick,
									&ArSick::
									getSickPacCount));
#endif

#ifdef ARNL_GPSLOC
  // TODO
#endif

Aria::getInfoGroup ()->addStringInt ("Motor Packet Count", 10,
				     new ArConstRetFunctorC < int,
				     ArRobot > (&robot,
						&ArRobot::getMotorPacCount));


  // Create service that allows client to change configuration parameters in ArConfig 
  ArServerHandlerConfig handlerConfig (&server, Aria::getConfig (),
				     Arnl::getTypicalDefaultParamFileName (),
				     Aria::getDirectory ());



  // Read in parameter files.
  Aria::getConfig ()->useArgumentParser (&parser);
  if (!Aria::getConfig ()->parseFile (Arnl::getTypicalParamFileName ()))
  {
    ArLog::log (ArLog::Normal, "Trouble loading configuration file, exiting");
    Aria::exit (5);
  }

  // Error if there is no map
  if (arMap.getFileName () == NULL || strlen (arMap.getFileName ()) <= 0)
  {
    ArLog::log(ArLog::Terse, "Warning, no map given. Use the -map command-line argument or modify the config using MobileEyes or by editing the parameter file.");
    ArLog::log(ArLog::Terse, "See the ARNL documentation, including MAPPING.txt or SONAR_MAPPING.txt.");
  }

  ArLog::log (ArLog::Normal, "Directory for maps and file serving: %s", fileDir);

  ArLog::log (ArLog::Normal, "See the ARNL README.txt for more information");


  robot.unlock();


  // Localize robot at home.
#ifdef ARNL_GPSLOC
 gpsLocTask.setIdleFlag(true);
#endif
#if defined(ARNL_LASERLOC) || defined(ARNL_SONARLOC)
  locTask.localizeRobotAtHomeBlocking();
#endif
  
  server.runAsync();

  ArLog::log(ArLog::Normal, "Server now running on port %d. Press Control-C to exit.", server.getTcpPort());


  robot.waitForRunExit();
  Aria::exit(0);

}

