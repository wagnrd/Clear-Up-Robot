/*
Adept MobileRobots Robotics Interface for Applications (ARIA)
Copyright (C) 2004, 2005 ActivMedia Robotics LLC
Copyright (C) 2006, 2007, 2008, 2009, 2010 MobileRobots Inc.
Copyright (C) 2011, 2012, 2013 Adept Technology

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation; either version 2 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

If you wish to redistribute ARIA under different terms, contact 
Adept MobileRobots for information about a commercial version of ARIA at 
robots@mobilerobots.com or 
Adept MobileRobots, 10 Columbia Drive, Amherst, NH 03031; +1-603-881-7960
*/
#include <Aria.h>

/**
@example actionExample.cpp  An example program demonstrating how to make and use new actions.

This example program creates two new actions, Go and Turn. Go will drive the robot forward safely,
while Turn will avoid obstacles detected by the sonar by turning. 
This program also adds a predefined
action from Aria which tries to recover from stalls (hit something and 
can't move forward) by backing and turning.

Each of these actions have the normal constructor and destructor, note that 
the constructors use constructor chaining to create their ArAction part
correctly.  Each action then also implements the essential virtual method, 
fire(). This fire function is called by the action resolver, and
returns values that, in combination with other actions' desired behavior,
determine the driving commands sent to the robot.

Also note that each of these actions override the setRobot function; these 
implementations obtain the sonar device from the robot in addition to doing the
needed caching of the robot pointer.  This is what you should do if you
care about the presence or absence of a particular sensor.  If you don't
care about any particular sensor you could just use one of the  checkRangeDevice...
methods in ArRobot (there are four of them).
Also note that these are very naive actions, they are simply an example
of how to use actions.

See the @ref actions Actions section of the Aria reference manual overview for more details about actions.

Note that actions must take a small amount of time to execute, to avoid
delaying the robot synchronization cycle.

*/
#include "actiongo.h"
#include "actionturn.h"


int main(int argc, char** argv)
{
  Aria::init();
  ArArgumentParser parser(&argc, argv);
  parser.loadDefaultArguments();
  ArRobot robot;
  ArSonarDevice sonar;

  // Connect to the robot, get some initial data from it such as type and name,
  // and then load parameter files for this robot.
  ArRobotConnector robotConnector(&parser, &robot);
  if(!robotConnector.connectRobot())
  {
    ArLog::log(ArLog::Terse, "actionExample: Could not connect to the robot.");
    if(parser.checkHelpAndWarnUnparsed())
    {
        // -help not given
        Aria::logOptions();
        Aria::exit(1);
    }
  }

  if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
  {
    Aria::logOptions();
    Aria::exit(1);
  }

  ArLog::log(ArLog::Normal, "actionExample: Connected to robot.");

  // Create instances of the actions defined above, plus ArActionStallRecover, 
  // a predefined action from Aria.
  ActionGo go(500, 350);
  ActionTurn turn(400, 10);
  ArActionStallRecover recover;

    
  // Add the range device to the robot. You should add all the range 
  // devices and such before you add actions
  robot.addRangeDevice(&sonar);

 
  // Add our actions in order. The second argument is the priority, 
  // with higher priority actions going first, and possibly pre-empting lower
  // priority actions.
  robot.addAction(&recover, 100);
  robot.addAction(&go, 50);
  robot.addAction(&turn, 49);

  // Enable the motors, disable amigobot sounds
  robot.enableMotors();

  // Run the robot processing cycle.
  // 'true' means to return if it loses connection,
  // after which we exit the program.
  robot.run(true);
  
  Aria::exit(0);
}
