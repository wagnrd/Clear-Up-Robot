#ifndef ROBOTINFOACTION_H
#define ROBOTINFOACTION_H

#include <ArAction.h>

class RobotInfoAction : public ArAction
{
public:
    RobotInfoAction();
    ArActionDesired *fire(ArActionDesired currentDesired);
};

#endif // ROBOTINFOACTION_H
