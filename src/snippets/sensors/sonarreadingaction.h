#ifndef SONARREADING_H
#define SONARREADING_H

#include <ArAction.h>

class SonarReadingAction : public ArAction
{
public:
    SonarReadingAction();
    ArActionDesired *fire(ArActionDesired currentDesired);
};

#endif // SONARREADING_H
