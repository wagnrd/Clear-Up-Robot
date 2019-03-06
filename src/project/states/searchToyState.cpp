#include "searchToyState.h"
#include "../config.h"
#include "../utils/utils.hpp"

void SearchToyState::enterState()
{
    ArLog::log(ArLog::Normal, "Entering SearchToyState");

    //check if last path planning was successfull (= we are at our correct position)
    if (!ConfigSingleton::getInstance()->lastPathPlanningSuccess)
    {
        ArLog::log(ArLog::Terse, "Last path planing was not successfull. Returning to home");

        myStateMachine->changeState(MainStateMachine::Action::GO_TO_HOME);
        return;
    }

    if (!initialized)
    {
        readingBoxX1 = getRobot()->getRobotRadius();
        readingBoxX2 = readingBoxX1 + 1000;
        readingBoxLeftY1 = 0;
        readingBoxLeftY2 = readingBoxLeftY1 + getRobot()->getRobotRadius() + 50;
        initialized = true;
    }
    goingToStartPos = true;

    //get laser
    laser = ConfigSingleton::getInstance()->laser;
    if (!laser->isConnected())
    {
        ArLog::log(ArLog::Normal, "Laser not connected but needed for searchToyState!");
        myStateMachine->changeState(MainStateMachine::Action::TERMINATE);
    }
    ConfigSingleton::getInstance()->camera->tilt(cameraTilt);
}

ArActionDesired *SearchToyState::updateState(const ArActionDesired &currentDesired)
{
    myDesired.reset();
    std::unique_ptr<ArACTSBlob> largestBlob = Utils::getLargestBlob(ConfigSingleton::getInstance()->targetChannel);
    if (largestBlob && largestBlob->getArea() > minBlobSize)
    {
        myDesired.setRotVel(0);
        myStateMachine->changeState(MainStateMachine::Action::FOCUS_ON_TOY);
        return &myDesired;
    }
    laser->lockDevice();
    if (goingToStartPos)
    {
        double distance = laser->currentReadingBox(readingBoxX1, readingBoxLeftY1, readingBoxX2, readingBoxLeftY2,
                                                   &reading);
        if (distance < distanceTreshold)
        {
            goingToStartPos = false;
            myDesired.setRotVel(0);
        }
        else
        {
            myDesired.setRotVel(rotationSpeed);
        }
    }
    else
    {
        double distance = laser->currentReadingBox(readingBoxX1, -readingBoxLeftY1, readingBoxX2, -readingBoxLeftY2,
                                                   &reading);

        if (distance < distanceTreshold)
        {
            //NOTHING FOUND
            myDesired.setRotVel(0);

            myStateMachine->changeState(MainStateMachine::Action::GO_TO_HOME);
        }
        else
        {
            myDesired.setRotVel(-rotationSpeed);
        }
    }
    laser->unlockDevice();
    return &myDesired;
}

void SearchToyState::exitState()
{
    ArLog::log(ArLog::Normal, "Exiting SearchToyState");
}
