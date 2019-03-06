#include "focusOnColorState.h"
#include "../config.h"
#include "../utils/utils.hpp"

ArActionDesired *FocusOnColorState::updateState(const ArActionDesired &currentDesired)
{
    Utils::TurnToColorResult result = Utils::turnToColor(ConfigSingleton::getInstance()->targetChannel, minBlobArea,
                                                         xAxisPid, yAxisPid);
    ArLog::log(ArLog::Normal, "CameraTilt: %f", camera->getTilt());
    if (!result.blobsFound)
    {
        //no blob in camera
        if (++framesWithoutBlob > 5) //some threshould because ACTS sometimes loses a blob for some reason
        {
            //no blobs found for some frames, searching
            ArLog::log(ArLog::Normal, "================RES==================");
            framesFocused = 0;

            //reset PID controllers
            xAxisPid.reset();
            yAxisPid.reset();

            //set motion to 0 (on purpose only here)
            myDesired.reset();
        }

        if (framesWithoutBlob > 20) //no blob detected for a long time, return to playarea
        {
            myStateMachine->changeState(MainStateMachine::Action::GO_TO_PLAY_AREA);
        }
    }
    //we should check that over multiple frames the object is centered
    else if (std::abs(result.correctionX) < xDiffTreshold)
    {
        //blob in camera and centered
        framesWithoutBlob = 0;
        if (++framesFocused == 10)
        {
            myStateMachine->changeState(MainStateMachine::Action::GO_TO_TOY);
        }
    }
    else
    {
        //blob in camera but not centered
        framesWithoutBlob = 0;
        myDesired.setDeltaHeading(result.correctionX);
        Utils::cameraTiltRel(result.correctionY);
        framesFocused = 0;
    }
    return &myDesired;
}

void FocusOnColorState::enterState()
{
    ArLog::log(ArLog::Normal, "Entering FocusOnColorState");
    acts = &ConfigSingleton::getInstance()->acts;
    camera = getRobot()->getPTZ();
    framesFocused = 0;
    framesWithoutBlob = 0;
}

void FocusOnColorState::exitState()
{
    ArLog::log(ArLog::Normal, "Exiting FocusOnColorState");
}
