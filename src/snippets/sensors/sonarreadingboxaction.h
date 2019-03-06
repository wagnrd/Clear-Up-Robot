#ifndef SONARREADINGBOX_H
#define SONARREADINGBOX_H

#include <ArAction.h>

class ArSonarDevice;

class SonarReadingBoxAction : public ArAction
{
public:
    SonarReadingBoxAction(ArSonarDevice *sonar);
    ArActionDesired *fire(ArActionDesired currentDesired);

protected:
    ArSonarDevice *sonar;
    ArPose readingPose;
    double x1, y1, x2, y2;

    void init();
    bool initialized;
};

#endif // SONARREADINGBOX_H
