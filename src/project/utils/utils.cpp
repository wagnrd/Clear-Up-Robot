#include "utils.hpp"
#include "ariaMath.hpp"
#include "../config.h"
#include "pid.hpp"

namespace Utils
{

bool goTo(ArActionDesired &desired, const ArPose &goToPose, bool stopAtArrival)
{
    ConfigSingleton *config = ConfigSingleton::getInstance();
    ArPose currentPose = config->robot->getPose();

    ArPose deltaVector = goToPose - currentPose;
    double newTh = AriaMath::getRotAngleDeg(deltaVector);
    double deltaLength = AriaMath::getLength(deltaVector);

    desired.setHeading(newTh);

    if (deltaLength > 200)
    {
        desired.setVel(300);
    }
    else if (deltaLength > 100)
    {
        desired.setVel(100);
    }
    else if (deltaLength > 50)
    {
        desired.setVel(50);
    }
    else if (deltaLength > 10 && stopAtArrival)
    {
        desired.setVel(20);
    }
    else
    {
        if (stopAtArrival)
        {
            desired.setVel(0);
        }

        return true;
    }

    return false;
}

bool goToRel(ArActionDesired &desired, const ArPose &goToPoseRel, bool stopAtArrival)
{
    ConfigSingleton *config = ConfigSingleton::getInstance();
    ArPose currentPose = config->robot->getPose();

    ArLog::log(ArLog::Normal, "x: %f, y: %f", currentPose.getX(), currentPose.getY());

    double newX = currentPose.getX() + goToPoseRel.getX();
    double newY = currentPose.getY() + goToPoseRel.getY();
    double newTh = currentPose.getTh() + goToPoseRel.getTh();
    ArPose goToPoseAbs(newX, newY, newTh);

    return goTo(desired, goToPoseAbs, stopAtArrival);
}

bool goToBackwards(ArActionDesired &desired, const ArPose &goToPose, bool stopAtArrival)
{
    ConfigSingleton *config = ConfigSingleton::getInstance();
    ArPose currentPose = config->robot->getPose();

    ArPose deltaVector = goToPose - currentPose;
    double newTh = AriaMath::getRotAngleDeg(ArPose(0, 0, 0) - deltaVector);
    double deltaLength = AriaMath::getLength(deltaVector);

    desired.setHeading(newTh);

    if (deltaLength > 100)
    {
        desired.setVel(-100);
    }
    else if (deltaLength > 50)
    {
        desired.setVel(-50);
    }
    else if (deltaLength > 10 && stopAtArrival)
    {
        desired.setVel(20);
    }
    else
    {
        if (stopAtArrival)
        {
            desired.setVel(0);
        }

        return true;
    }

    return false;
}

bool goToBackwardsRel(ArActionDesired &desired, const ArPose &goToPoseRel, bool stopAtArrival)
{
    ConfigSingleton *config = ConfigSingleton::getInstance();
    ArPose currentPose = config->robot->getPose();

    double newX = currentPose.getX() + goToPoseRel.getX();
    double newY = currentPose.getY() + goToPoseRel.getY();
    double newTh = currentPose.getTh() + goToPoseRel.getTh();
    ArPose goToPoseAbs(newX, newY, newTh);

    return goToBackwards(desired, goToPoseAbs, stopAtArrival);
}

bool goUntilObstacleIn(ArActionDesired &desired, double distance, double speed)
{
    ConfigSingleton *config = ConfigSingleton::getInstance();
    ArLaser *laser = config->laser;

    double robotRadius = config->robot->getRobotRadius();
    double x1 = robotRadius;
    double y1 = robotRadius;
    double x2 = 32000;
    double y2 = -robotRadius;

    double obstacleDistance = laser->currentReadingBox(x1, y1, x2, y2);
    ArLog::log(ArLog::Normal, "Distance: %f, ObstacleDistance: %f", distance, obstacleDistance);

    if (obstacleDistance < robotRadius + distance)
    {
        desired.setVel(0);
        return true;
    }
    else
    {
        desired.setVel(speed);
        return false;
    }
}

bool goUntilObstacleGoneIn(ArActionDesired &desired, double distance, double speed)
{
    ConfigSingleton *config = ConfigSingleton::getInstance();
    ArLaser *laser = config->laser;

    double robotRadius = config->robot->getRobotRadius();
    double x1 = robotRadius;
    double y1 = robotRadius;
    double x2 = 32000;
    double y2 = -robotRadius;

    double obstacleDistance = laser->currentReadingBox(x1, y1, x2, y2);
    ArLog::log(ArLog::Normal, "Distance: %f, ObstacleDistance: %f", distance, obstacleDistance);

    if (obstacleDistance < robotRadius + distance)
    {
        desired.setVel(speed);
        return false;
    }
    else
    {
        desired.setVel(0);
        return true;
    }
}

TurnToColorResult turnToColor(int channel, double minArea, PidController<double> &pidX, PidController<double> &pidY)
{
    ConfigSingleton *config = ConfigSingleton::getInstance();

    if (!config->robot)
    {
        ArLog::log(ArLog::Terse, "TTC: myRobot is null!!");
    }

    if (!config->acts.isConnected())
    {
        ArLog::log(ArLog::Terse, "TTC: Not connected to ACTS");
        return TurnToColorResult(); //TODO think about returning error
    }

    std::unique_ptr<ArACTSBlob> largestBlob = getLargestBlob(channel);

    if (!largestBlob)
    {
        ArLog::log(ArLog::Normal, "TTC: No Blobs found");
        return TurnToColorResult();
    }

    if (largestBlob->getArea() < minArea)
    {
        ArLog::log(ArLog::Normal, "TTC: Blobs found, but too small");
        return TurnToColorResult();
    }

    //calculate x and y distance to middle of the screen
    double xDiffPercent = ((static_cast<double>(config->cameraWith) / 2) - largestBlob->getXCG()) / config->cameraWith * 100;
    double yDiffPercent = ((static_cast<double>(config->cameraHeight) / 2) - largestBlob->getYCG()) / config->cameraHeight * 100;

    //correct x
    double xCorrection = pidX.update(xDiffPercent);

    //correct y
    double yCorrection = pidY.update(yDiffPercent);

    ArLog::log(ArLog::Normal, "TTC: Tracking blob at: %d %d", largestBlob->getXCG(), largestBlob->getYCG());
    ArLog::log(ArLog::Normal, "TTC: xDiff: %.2f, yDiff: %.2f", xDiffPercent, yDiffPercent);
    ArLog::log(ArLog::Normal, "TTC: suggested correction: x: %.2f, y: %.2f", xCorrection, yCorrection);

    return TurnToColorResult(xDiffPercent, yDiffPercent, xCorrection, yCorrection);
}

std::unique_ptr<ArACTSBlob> getLargestBlob(int targetChannel)
{
    ConfigSingleton *config = ConfigSingleton::getInstance();

    int blobsCount = config->acts.getNumBlobs(targetChannel);
    ArLog::log(ArLog::Normal, "GLB: BlobCount: %d on channel %d", blobsCount, targetChannel);

    if (blobsCount == 0)
    {
        ArLog::log(ArLog::Normal, "GLB: No Blobs found");
        return nullptr;
    }

    double largestBlobArea = 0;

    bool targetFound = false; //targetBlob is no pointer, cant set to nullptr
    ArACTSBlob targetBlob = ArACTSBlob();
    ArACTSBlob tmpBlob = ArACTSBlob();

    for (int i = 1; i <= blobsCount; i++) //acts starts counting at 1...
    {
        if (!config->acts.getBlob(targetChannel, i, &tmpBlob))
        {
            continue; //continue if getBlob tells me this blob is not available for some reason
        }
        if (tmpBlob.getArea() > largestBlobArea)
        {
            targetBlob = tmpBlob;
            largestBlobArea = tmpBlob.getArea();
            targetFound = true;
        }
    }

    if (targetFound)
    {
        ArLog::log(ArLog::Normal, "GLB: largest Blob: %d", blobsCount, targetBlob.getArea());
        return std::make_unique<ArACTSBlob>(targetBlob);
    }

    ArLog::log(ArLog::Normal, "GLB: No Blobs found2");
    return nullptr;
}

// Gripper utils
void gripperLiftUp()
{
    ConfigSingleton *config = ConfigSingleton::getInstance();

    if (!config->isGripperUp)
    {
        config->isGripperUp = true;
        config->gripper->liftUp();
    }
}

void gripperLiftDown()
{
    ConfigSingleton *config = ConfigSingleton::getInstance();

    if (config->isGripperUp)
    {
        config->isGripperUp = false;
        config->gripper->liftDown();
    }
}

void gripperClose()
{
    ConfigSingleton *config = ConfigSingleton::getInstance();

    if (config->isGripperOpen)
    {
        config->isGripperOpen = false;
        config->gripper->gripClose();
    }
}

void gripperOpen()
{
    ConfigSingleton *config = ConfigSingleton::getInstance();

    if (!config->isGripperOpen)
    {
        config->isGripperOpen = true;
        config->gripper->gripOpen();
    }
}

void gripperStore()
{
    gripperClose();
    gripperLiftUp();
}

void gripperDeploy()
{
    gripperLiftDown();
    gripperOpen();
}

// Camera utils
void cameraTilt(double angle)
{
    ConfigSingleton *config = ConfigSingleton::getInstance();

    config->camera->tilt(clamp(angle, -27.0, -10.0));
}

void cameraTiltRel(double angle)
{
    ConfigSingleton *config = ConfigSingleton::getInstance();

    config->camera->tilt(clamp(config->camera->getTilt() + angle, -27.0, -10.0));
}

} // namespace Utils