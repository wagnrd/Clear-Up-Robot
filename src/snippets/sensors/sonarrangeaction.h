#ifndef SONARRANGEACTION_H
#define SONARRANGEACTION_H

#include <ArAction.h>

class SonarRangeAction : public ArAction
{
public:
    SonarRangeAction();
    ArActionDesired *fire(ArActionDesired currentDesired);
};

#endif // SONARRANGEACTION_H
