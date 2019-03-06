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
#include "Aria.h"
#include "ArNetworking.h"
#include "Arnl.h"


class SimpleTask
{
public:
  SimpleTask(ArRobot *robot, ArPathPlanningTask *pathTask);
  virtual ~SimpleTask();
  void task(void);
  void goalDone(ArPose pose);
protected:
  ArRobot *myRobot;
  ArPathPlanningTask *myPathTask;

  int myState;
  bool myGoalDone;
  ArPose myHome;
  ArFunctorC<SimpleTask> myTaskCB;
  ArFunctor1C<SimpleTask, ArPose> myGoalDoneCB;
};

SimpleTask::SimpleTask(ArRobot *robot, ArPathPlanningTask *pathTask) :
  myTaskCB(this, &SimpleTask::task),
  myGoalDoneCB(this, &SimpleTask::goalDone)
{
  myRobot = robot;
  myPathTask = pathTask;

  // trigger the first start off
  myGoalDone = true;
  myState = 0;
  myHome = robot->getPose();
  
  myRobot->addUserTask("simpleTask", 50, &myTaskCB);
  myPathTask->addGoalDoneCB(&myGoalDoneCB);
  myPathTask->addGoalFailedCB(&myGoalDoneCB);
}

SimpleTask::~SimpleTask()
{
  
}

void SimpleTask::goalDone(ArPose pose)
{
  myGoalDone = true;
}

void SimpleTask::task(void)
{
  if (!myGoalDone)
    return;
  
  myGoalDone = false;
  myState++;
  if (myState == 1)
    myPathTask->pathPlanToPose(ArPose(-1000, -1000, 0), true);
  //else if (myState == 1)
  //myPathTask->pathPlanToGoal("Goal1");
  else 
  {
    myState = 0;
    myPathTask->pathPlanToPose(myHome, true);
  }
}

int 
main(int argc, char *argv[])
{
  Aria::init();
  Arnl::init();
  
  ArRobot robot;
  ArSick sick(181, 361);

  ArArgumentParser parser(&argc, argv);
  ArSimpleConnector simpleConnector(&parser);

  parser.loadDefaultArguments();
  simpleConnector.parseArgs();

  // Check for --help
  if (!parser.checkHelpAndWarnUnparsed())
  {
    printf("\nUsage: simpleDemo [-map <map file>] [other options]\n\nIf -map not given, use map from ARNL configuration file (%s).\n\n", Arnl::getTypicalParamFileName());
    simpleConnector.logOptions();
    Aria::exit(4);
  }

  ArSonarDevice sonarDev;
  robot.addRangeDevice(&sonarDev);

  ArMap arMap;

#ifdef ARNL
  // Initialize the localization
  ArLocalizationTask locTask(&robot, &sick, &arMap);
  // Make the path task
  ArPathPlanningTask pathTask(&robot, &sick, &sonarDev, &arMap);
#elif defined(SONARNL)
  // Initialize the localization
  ArSonarLocalizationTask locTask(&robot, &sonarDev, &arMap);
  // Make the path task
  ArPathPlanningTask pathTask(&robot, NULL, &sonarDev, &arMap);
#else 
  #error simpleDemo.cpp currently only works with ARNL and SONARNL.
#endif


  // Stop the robot as soon as localization fails.
  ArFunctor1C<ArPathPlanningTask, int> 
  locaFailed(&pathTask, &ArPathPlanningTask::trackingFailed);
  locTask.addFailedLocalizationCB(&locaFailed);
  
  // Read ARNL parameter file
  Aria::getConfig()->useArgumentParser(&parser);
  if (Aria::getConfig()->parseFile(Arnl::getTypicalParamFileName()))
  {
    printf("Loaded configuration file %s\n", Arnl::getTypicalParamFileName());
  }
  else
  {
    printf("Trouble loading configuration file, exiting\n");
    Aria::exit(5);
  }

  // Connect to the robot
  if (!simpleConnector.connectRobot(&robot))
  {
    printf("Could not connect to robot... exiting\n");
    Aria::shutdown();
    Aria::exit(1);
  }


#ifdef ARNL
  // Set up the laser before handing it to the laser mode
  simpleConnector.setupLaser(&sick);

  // Add the laser to the robot
  robot.addRangeDevice(&sick);
#endif

  // Start the robot thread.
  robot.runAsync(true);

#ifdef ARNL  
  // Start the laser thread.
  sick.runAsync();

  // Connect the laser
  if (!sick.blockingConnect())
  {
    printf("Couldn't connect to sick, exiting\n");
    Aria::shutdown();
    Aria::exit(2);
  }
#endif

  ArUtil::sleep(300);

  // Localize the robot to home
  if(locTask.localizeRobotAtHomeBlocking()) 
  {
    printf("Successfully localized at home.\n");
  } 
  else 
  {
    printf("Error: Unable to localize in map (based on home position)!\n");
    Aria::exit(3);
  }

  // Instance of SimpleTask class defined above to manage the simple motion
  robot.lock();
  SimpleTask simpleTask(&robot, &pathTask);
  robot.unlock();

  // Enable motors and Wait for program cancelled or robot connection lost
  robot.lock();
  robot.enableMotors();
  robot.unlock();
  robot.waitForRunExit();
  exit(0);

}
