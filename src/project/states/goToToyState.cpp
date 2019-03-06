#include "goToToyState.h"
#include "../utils/ariaMath.hpp"
#include "../config.h"
#include "../utils/utils.hpp"
#include <ArACTS.h>
#include <memory>

void GoToToyState::enterState()
{
    ArLog::log(ArLog::Normal, "Entering GoToToyState");

    gripper = ConfigSingleton::getInstance()->gripper;
    Utils::gripperDeploy();
    acts = &ConfigSingleton::getInstance()->acts;
    blobNotFoundCounter = 0;
    distanceMedianMM = 0;
    camera = getRobot()->getPTZ();
    tooCloseLeft = false;
    tooCloseRight = false;
    tooCloseCounter = 0;
    frameWithoutBlobCounter = 5;
    cantAccessToyCounter = 0;
    ignoreWrongToy = false;
    wrongToyBehindCounter = 0;
    toyBrokeOuterBeam = false;
    toyInGripperCounter = 0;
}

ArActionDesired *GoToToyState::updateState(const ArActionDesired &currentDesired)
{
    ConfigSingleton *config = ConfigSingleton::getInstance();

    std::unique_ptr<ArACTSBlob> largestBlob = Utils::getLargestBlob(config->targetChannel);

    if (largestBlob)
    {
        blobNotFoundCounter = 0;

        // controls steering
        Utils::TurnToColorResult result = Utils::turnToColor(config->targetChannel, minBlobArea,
                                                             xAxisPid, yAxisPid);

        myDesired.setDeltaHeading(result.correctionX); // steers
        Utils::cameraTiltRel(result.correctionY);      // tilts camera

        // Camera 1: (top, left) 116, 216 | (bottom, right) 311, 512 -> blobHeight: 195px
        const double focalLengthPX = config->focalLength;
        const double objectHeightMM = 80;
        double blobHeightPX = largestBlob->getBottom() - largestBlob->getTop();

        //constexpr double objectBreadthMM = 110;
        //constexpr double distanceToFrontMM = 230;
        //constexpr double distanceToMiddleMM = 290;

        // Intercept theorem (Strahlensatz)
        double currentDistanceMM = (focalLengthPX * objectHeightMM) / blobHeightPX;
        distanceMedianMM = ((1.1 * currentDistanceMM + 0.9 * distanceMedianMM) / 2);

        ArLog::log(ArLog::Normal, "GTT: currentX: %f currentY: %f, medianDistance: %f", getRobot()->getX(), getRobot()->getY(), distanceMedianMM);

        if (currentDistanceMM > 150)
        {
            //ArLog::log(ArLog::Normal, "GTT: currentDistanceMM >150");
            myDesired.setVel(20);
        }
        else if (currentDistanceMM > 100)
        {
            //ArLog::log(ArLog::Normal, "GTT: currentDistanceMM >100");
            myDesired.setVel(150);
        }
        else
        {
            myDesired.setVel(80);
        }
    }
    else
    {
        // Default velocity if nothing else is given
        myDesired.setVel(60);
    }

    // changing to goToPlayArea if no Blobs were found for an amount of iterations (5 secs)
    if (!toyBrokeOuterBeam && ++blobNotFoundCounter >= 100)
    {
        ArLog::log(ArLog::Normal, "GTT: blobNotFoundCounter >= 50");
        myDesired.setVel(0);
        Utils::gripperStore();
        myStateMachine->changeState(MainStateMachine::Action::GO_TO_PLAY_AREA);
    }

    // check whether toy is inside the gripper paddles
    if ((gripper->getBreakBeamState() == 3 && gripper->getGripState() == 1) || (toyBrokeOuterBeam && ++toyInGripperCounter >= 50))
    {
        ArLog::log(ArLog::Normal, "GTT: Toy inside the paddles");
        myDesired.setVel(0);
        myStateMachine->changeState(MainStateMachine::Action::TAKE_TOY);
    }
    else if (gripper->getBreakBeamState() == 2 && gripper->getGripState() == 1)
    {
        myDesired.setVel(30);
        toyBrokeOuterBeam = true;
    }

    // uses laserReadingBoxes to ensure to not bump into any object in front of the robot
    double robotRadius = getRobot()->getRobotRadius();

    double leftX1 = robotRadius;
    double leftY1 = robotRadius + maxSideDistance;
    double leftX2 = robotRadius + maxFrontDistance;
    double leftY2 = 0;
    ArPose leftReading;

    double rightX1 = robotRadius;
    double rightY1 = 0;
    double rightX2 = robotRadius + maxFrontDistance;
    double rightY2 = -(robotRadius + maxSideDistance);
    ArPose rightReading;

    double frontX1 = robotRadius;
    double frontY1 = robotRadius + maxSideDistance;
    double frontX2 = robotRadius + maxFrontDistance;
    double frontY2 = -(robotRadius + maxSideDistance);
    ArPose frontReading;

    double leftDistance = config->laser->currentReadingBox(leftX1, leftY1, leftX2, leftY2, &leftReading);
    double rightDistance = config->laser->currentReadingBox(rightX1, rightY1, rightX2, rightY2, &rightReading);
    double frontDistance = config->laser->currentReadingBox(frontX1, frontY1, frontX2, frontY2, &frontReading);

    if (leftDistance != config->laser->getMaxRange() &&
        leftReading.getX() < leftX2 && leftReading.getY() < leftY1)
    {
        ArLog::log(ArLog::Normal, "Obstacle too close on the left");
        tooCloseLeft = true;
    }
    else if (tooCloseLeft)
    {
        tooCloseLeft = false;
    }

    if (rightDistance != config->laser->getMaxRange() &&
        rightReading.getX() < rightX1 && rightReading.getY() > rightY2)
    {
        ArLog::log(ArLog::Normal, "Obstacle too close on the right");
        tooCloseRight = true;
    }
    else if (tooCloseRight)
    {
        tooCloseRight = false;
    }

    if (frontDistance != config->laser->getMaxRange() && frontReading.getX() < frontX2)
    {
        ArLog::log(ArLog::Normal, "Obstacle too close in the front");
        ArLog::log(ArLog::Normal, "x: %f, y: %f, distance: %f", frontReading.getX(), frontReading.getY(), frontDistance);
        myDesired.setVel(0);

        if (++tooCloseCounter > 80)
        {
            myStateMachine->changeState(MainStateMachine::Action::GO_TO_PLAY_AREA);
            tooCloseCounter = 0;
        }
    }

    // ensures, that no other large blob is found in a specific area in front of the robot and the robot will stop
    // if the other large blob is not removed, the robot will fall back to goToHomeState
    for (int channel = 1; channel <= acts->NUM_CHANNELS; channel++)
    {
        if (channel == config->targetChannel)
        {
            continue;
        }

        int blobsCount = acts->getNumBlobs(channel);
        ArACTSBlob blob;

        for (int i = 1; i <= blobsCount; i++)
        {
            if (!acts->getBlob(channel, i, &blob))
            {
                continue;
            }

            if (blob.getArea() >= minBlobArea &&
                blob.getXCG() >= minCameraAreaX && blob.getXCG() <= maxCameraAreaX &&
                blob.getYCG() >= minCameraAreaY && blob.getYCG() <= maxCameraAreaY)
            {
                frameWithoutBlobCounter = 0;

                if (largestBlob && largestBlob->getYCG() > blob.getYCG())
                {
                    ignoreWrongToy = true;
                }
            }
        }
    }

    if (!ignoreWrongToy && ++frameWithoutBlobCounter < 5)
    {
        ArLog::log(ArLog::Normal, "Wrong toy in drive way");
        myDesired.setVel(0);

        if (++cantAccessToyCounter > 150)
        {
            Utils::gripperStore();
            myStateMachine->changeState(MainStateMachine::Action::GO_TO_HOME);
        }
    }
    else
    {
        frameWithoutBlobCounter = 5;
    }

    return &myDesired;
}

void GoToToyState::exitState()
{
    ArLog::log(ArLog::Normal, "Exiting GoToToyState");
}
