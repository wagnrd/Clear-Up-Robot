#include "actionturn.h"



/*
  This is the ActionTurn constructor, note the use of constructor chaining
  with the ArAction. also note how it uses setNextArgument, which makes
  it so that other things can see what parameters this action has, and
  set them.  It also initializes the classes variables.
*/
ActionTurn::ActionTurn(double turnThreshold, double turnAmount) :
  ArAction("Turn")
{
  myTurnThreshold = turnThreshold;
  myTurnAmount = turnAmount;
  setNextArgument(ArArg("turn threshold (mm)", &myTurnThreshold, "The number of mm away from obstacle to begin turnning."));
  setNextArgument(ArArg("turn amount (deg)", &myTurnAmount, "The number of degress to turn if turning."));
  myTurning = 0;
}

/*
  Sets the myRobot pointer (all setRobot overloaded functions must do this),
  finds the sonar device from the robot, and if the sonar isn't there,
  then it deactivates itself.
*/
void ActionTurn::setRobot(ArRobot *robot)
{
  ArAction::setRobot(robot);
  mySonar = robot->findRangeDevice("sonar");
  if (mySonar == NULL)
  {
    ArLog::log(ArLog::Terse, "actionExample: ActionTurn: Warning: I found no sonar, deactivating.");
    deactivate();
  }
}

/*
  This is the guts of the Turn action.
*/
ArActionDesired *ActionTurn::fire(ArActionDesired currentDesired)
{
  double leftRange, rightRange;
  // reset the actionDesired (must be done)
  myDesired.reset();
  // if the sonar is null we can't do anything, so deactivate
  if (mySonar == NULL)
  {
    deactivate();
    return NULL;
  }
  // Get the left readings and right readings off of the sonar
  leftRange = (mySonar->currentReadingPolar(0, 100) -
        myRobot->getRobotRadius());
  rightRange = (mySonar->currentReadingPolar(-100, 0) -
        myRobot->getRobotRadius());
  // if neither left nor right range is within the turn threshold,
  // reset the turning variable and don't turn
  if (leftRange > myTurnThreshold && rightRange > myTurnThreshold)
  {
    myTurning = 0;
    myDesired.setDeltaHeading(0);
  }
  // if we're already turning some direction, keep turning that direction
  else if (myTurning)
  {
    myDesired.setDeltaHeading(myTurnAmount * myTurning);
  }
  // if we're not turning already, but need to, and left is closer, turn right
  // and set the turning variable so we turn the same direction for as long as
  // we need to
  else if (leftRange < rightRange)
  {
    myTurning = -1;
    myDesired.setDeltaHeading(myTurnAmount * myTurning);
  }
  // if we're not turning already, but need to, and right is closer, turn left
  // and set the turning variable so we turn the same direction for as long as
  // we need to
  else
  {
    myTurning = 1;
    myDesired.setDeltaHeading(myTurnAmount * myTurning);
  }
  // return a pointer to the actionDesired, so resolver knows what to do
  return &myDesired;
}
