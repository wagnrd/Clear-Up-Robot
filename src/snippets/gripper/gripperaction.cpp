#include "gripperaction.h"

#include <ArGripper.h>

GripperAction::GripperAction(ArGripper *gripper) :
    ArAction("GripperAction",
             "A sample Action demonstrating the use of the Gripper"),
    gripper(gripper),
    counter(0)
{
}

std::string GripperAction::BREAK_BEAM_STATE_NAMES[] = {"no", "inner", "outer", "both"};
std::string GripperAction::GRIP_STATE_NAMES[] = {"between", "open", "closed"};
std::string GripperAction::PADDLE_STATE_NAMES[] = {"no", "left", "right", "both"};

ArActionDesired *GripperAction::fire(ArActionDesired currentDesired) {

    if(!gripper) {
        ArLog::log(ArLog::Terse, "Warning: A gripper is needed for the GripperAction!");
        deactivate();
        return 0;
    }

    // 0 if no, 1 if inner, 2 if outter, 3 if both breakbeams are broken
    int bbState = gripper->getBreakBeamState();
    ArLog::log(ArLog::Normal, "BreakBeam State: %d %s", bbState, BREAK_BEAM_STATE_NAMES[bbState].c_str());

    // 0 if gripper paddles are between open and closed, 1 if open, 2 closed
    int gripState = gripper->getGripState();
    ArLog::log(ArLog::Normal, "Grip State: %d %s", gripState, GRIP_STATE_NAMES[gripState].c_str());

    // 0 if no, 1 if left, 2 if right, 3 if both gripper paddles are triggered
    int paddleState = gripper->getPaddleState();
    ArLog::log(ArLog::Normal, "Paddle State: %d %s", paddleState, PADDLE_STATE_NAMES[paddleState].c_str());

    int interval = 200;

    int counterMod = counter % interval;

    if (0 == counterMod) {
        ArLog::log(ArLog::Normal, "Gripper store!");
        gripper->gripperStore();
    } else if (interval/2 == counterMod) {
        ArLog::log(ArLog::Normal, "Gripper deploy!");
        gripper->gripperDeploy();
    }

    ++counter;

    return 0;
}
