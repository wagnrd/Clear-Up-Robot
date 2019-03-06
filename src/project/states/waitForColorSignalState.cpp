#include "waitForColorSignalState.h"
#include "../config.h"
#include <Aria.h>

ArActionDesired *WaitForColorSignalState::updateState(const ArActionDesired &currentDesired)
{
    if (!acts->isConnected())
    {
        ArLog::log(ArLog::Terse, "Not connected to ACTS.");
        myStateMachine->changeState(MainStateMachine::Action::TERMINATE);
    }
    myDesired.reset();

    //start laser for other tasks
    ArLaser *laser = ConfigSingleton::getInstance()->laser;
    if (!laser)
    {
        ArLog::log(ArLog::Terse, "No Laser in ConfigSingleton!");
        myStateMachine->changeState(MainStateMachine::Action::TERMINATE);
    }
    laser->lockDevice();
    if (!laser->isConnected())
    {
        if (laser->isTryingToConnect())
        {
            //ArLog::log(ArLog::Normal, "=====LASER WARM UP, PLEASE WAIT=====");
        }
        else
        {
            laser->asyncConnect();
            ArLog::log(ArLog::Normal, "Connecting to Laser %s, please wait.",
                       laser->getName());
        }
        laser->unlockDevice();
        return &myDesired;
    }
    laser->unlockDevice();
    ArLog::log(ArLog::Normal, "Laser online");

    ConfigSingleton::getInstance()->camera->tilt(-10);
    for (int i = 1; i <= acts->NUM_CHANNELS; i++) //ACTS start counting from 1
    {
        blobsCount = acts->getNumBlobs(i);
        largestBlobArea = 0;
        for (int j = 1; j <= blobsCount; j++)
        {
            acts->getBlob(i, j, &tmpBlob);
            if (tmpBlob.getArea() > largestBlobArea)
            {
                largestBlobArea = tmpBlob.getArea();
            }
        }
        if (blobsCount > 0)
        {
            ArLog::log(ArLog::Normal, "Channel %d: numBlobs: %d, largestBlob: %d", i, blobsCount, largestBlobArea);
        }
        if (largestBlobArea >= minBlobArea)
        {
            ConfigSingleton::getInstance()->targetChannel = i;
            ArLog::log(ArLog::Normal, "Large Blob found, target channel set to %d", i);

            myStateMachine->changeState(nextState);

            break;
        }
    }

    return &myDesired;
}

void WaitForColorSignalState::enterState()
{
    ArLog::log(ArLog::Normal, "Entering WaitForColorSignalState");
    acts = &ConfigSingleton::getInstance()->acts;
    if(!ConfigSingleton::getInstance()->lastPathPlanningSuccess)
    {
        ArLog::log(ArLog::Terse, "Last path planing was not successfull. No Fallback for going to home, terminating...");
        myStateMachine->changeState(MainStateMachine::Action::TERMINATE);
    }
}

void WaitForColorSignalState::exitState()
{
    ArLog::log(ArLog::Normal, "Exiting WaitForColorSignalState");
}