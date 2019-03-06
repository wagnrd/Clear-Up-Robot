#include "cameraaction.h"

#include <Aria.h>

CameraAction::CameraAction() :
    ArAction("CameraAction",
             "A sample Action to demonstrate the use of the Camera."),
    counter(0),
    direction(1)
{
}

ArActionDesired *CameraAction::fire(ArActionDesired currentDesired)
{
    if(myRobot) {
        camera = myRobot->getPTZ();
    }
    if(!camera) {
        ArLog::log(ArLog::Terse, "Warning: A camera is needed for the CameraAction!");
        deactivate();
        return 0;
    }

    if (counter == 0) {
        camera->pan(0);
    }

    if (++counter % 50 == 0) {
        ArLog::log(ArLog::Normal, "pan: %d", 25*direction);
        camera->pan(25*direction);
        direction *= -1;
    }

    return 0;
}
