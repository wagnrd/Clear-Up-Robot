#pragma once

#include <Aria.h>
#include <memory>
#include <assert.h>

/*
 * Contains util functions for movement, steering, etc.
 * 
 * AUTHOR:
 * clamp, turnTocolor, TurnToColorResult & getLargestBlob: Jonas Eckstein
 * everything else: Denis Wagner
 */

namespace Utils
{
//forward declaration
template <typename T>
class PidController;

struct TurnToColorResult
{
    bool blobsFound;
    double xDifferencePercent;
    double yDifferencePercent;
    double correctionX;
    double correctionY;
    TurnToColorResult() : blobsFound(false), xDifferencePercent(-1), yDifferencePercent(-1), correctionX(0), correctionY(0) {}
    TurnToColorResult(double xDifferencePercent, double yDifferencePercent,
                      double correctionX, double correctionY) : blobsFound(true), xDifferencePercent(xDifferencePercent),
                                                                yDifferencePercent(yDifferencePercent), correctionX(correctionX),
                                                                correctionY(correctionY) {}
};

template <class T>
const T &clamp(const T &v, const T &min, const T &max)
{
    assert(min <= max);
    if (v < min)
    {
        return min;
    }
    else if (v > max)
    {
        return max;
    }
    else
    {
        return v;
    }
}

std::unique_ptr<ArACTSBlob> getLargestBlob(int targetChannel);
TurnToColorResult turnToColor(int channel, double minArea, PidController<double> &pidX, PidController<double> &pidY);

// Motion utils
bool goTo(ArActionDesired &desired, const ArPose &goToPose, bool stopAtArrival = true);
bool goToRel(ArActionDesired &desired, const ArPose &goToPoseRel, bool stopAtArrival = true);
bool goToBackwards(ArActionDesired &desired, const ArPose &goToPoseRel, bool stopAtArrival = true);
bool goToBackwardsRel(ArActionDesired &desired, const ArPose &goToPoseRel, bool stopAtArrival = true);
bool goUntilObstacleIn(ArActionDesired &desired, double distance, double speed);
bool goUntilObstacleGoneIn(ArActionDesired &desired, double distance, double speed);

// Gripper utils
void gripperLiftUp();
void gripperLiftDown();
void gripperClose();
void gripperOpen();
void gripperStore();
void gripperDeploy();

// Camera utils
void cameraTilt(double angle);
void cameraTiltRel(double angle);

} // namespace Utils
