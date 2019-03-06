#ifndef GRIPPERACTION_H
#define GRIPPERACTION_H

#include <ArAction.h>
#include <string>

class ArGripper;

class GripperAction : public ArAction
{
public:
    GripperAction(ArGripper *gripper);
    ArActionDesired *fire(ArActionDesired currentDesired);

    static std::string BREAK_BEAM_STATE_NAMES[];
    static std::string GRIP_STATE_NAMES[];
    static std::string PADDLE_STATE_NAMES[];

protected:
    ArGripper *gripper;
    int counter;
};

#endif // GRIPPERACTION_H
