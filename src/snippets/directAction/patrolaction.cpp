#include "patrolaction.h"

#include <Aria.h>

PatrolAction::PatrolAction() :
    ArAction("PatrolAction",
             "This action will let the robot patrol back and forth."),
    state(Move)
{
}

ArActionDesired *PatrolAction::fire(ArActionDesired currentDesired)
{
    if (myRobot->isMoveDone() && myRobot->isHeadingDone()) {
        switch (state) {
        case Move:
            myRobot->move(1000);
            state = Turn;
            break;
        case Turn:
            myRobot->setDeltaHeading(180);
            state = Move;
            break;
        }
    }
    return 0;
}
