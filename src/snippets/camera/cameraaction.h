#ifndef CAMERAACTION_H
#define CAMERAACTION_H

#include <ArAction.h>

class ArPTZ;

class CameraAction : public ArAction
{
public:
    CameraAction();
    ArActionDesired *fire(ArActionDesired currentDesired);

protected:
    ArPTZ *camera;
    int counter;
    int direction;
};

#endif // CAMERAACTION_H
