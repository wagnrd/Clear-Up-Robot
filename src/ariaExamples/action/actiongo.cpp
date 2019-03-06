#include "actiongo.h"

/*
  Note the use of constructor chaining with
  ArAction(actionName). Also note how it uses setNextArgument, which makes it so that
  other parts of the program could find out what parameters this action has, and possibly modify them.
*/
ActionGo::ActionGo(double maxSpeed, double stopDistance) :
  ArAction("Go")
{
  mySonar = NULL;
  myMaxSpeed = maxSpeed;
  myStopDistance = stopDistance;
  setNextArgument(ArArg("maximum speed", &myMaxSpeed, "Maximum speed to go."));
  setNextArgument(ArArg("stop distance", &myStopDistance, "Distance at which to stop."));
}

/*
  Override ArAction::setRobot() to get the sonar device from the robot, or deactivate this action if it is missing.
  You must also call ArAction::setRobot() to properly store
  the ArRobot pointer in the ArAction base class.
*/
void ActionGo::setRobot(ArRobot *robot)
{
  ArAction::setRobot(robot);
  mySonar = robot->findRangeDevice("sonar");
  if (robot == NULL)
    {
      ArLog::log(ArLog::Terse, "actionExample: ActionGo: Warning: I found no sonar, deactivating.");
      deactivate();
    }
}

/*
  This fire is the whole point of the action.
  currentDesired is the combined desired action from other actions
  previously processed by the action resolver.  In this case, we're
  not interested in that, we will set our desired
  forward velocity in the myDesired member, and return it.

  Note that myDesired must be a class member, since this method
  will return a pointer to myDesired to the caller. If we had
  declared the desired action as a local variable in this method,
  the pointer we returned would be invalid after this method
  returned.
*/
ArActionDesired *ActionGo::fire(ArActionDesired currentDesired)
{
  double range;
  double speed;

  // reset the actionDesired (must be done), to clear
  // its previous values.
  myDesired.reset();

  // if the sonar is null we can't do anything, so deactivate
  if (mySonar == NULL)
  {
    deactivate();
    return NULL;
  }
  // get the range of the sonar
  range = mySonar->currentReadingPolar(-70, 70) - myRobot->getRobotRadius();
  // if the range is greater than the stop distance, find some speed to go
  if (range > myStopDistance)
  {
    // just an arbitrary speed based on the range
    speed = range * .3;
    // if that speed is greater than our max, cap it
    if (speed > myMaxSpeed)
      speed = myMaxSpeed;
    // now set the velocity
    myDesired.setVel(speed);
  }
  // the range was less than the stop distance, so request stop
  else
  {
    myDesired.setVel(0);
  }
  // return a pointer to the actionDesired to the resolver to make our request
  return &myDesired;
}
