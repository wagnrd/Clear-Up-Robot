#include "actsaction.h"

#include <Aria.h>

ActsAction::ActsAction() :
    ArAction("ActsAction",
             "A sample action to demonstrate ACTS.")
{
}

ActsAction::~ActsAction()
{
    if (acts.isConnected()) {
        acts.closePort();
    }
}

ArActionDesired *ActsAction::fire(ArActionDesired currentDesired)
{
    if (acts.isConnected()) {
        for (int channel = 1; channel <= acts.NUM_CHANNELS; ++channel)
        {
            printChannel(channel);
        }
    } else {
        ArLog::log(ArLog::Normal, "Not connected to ACTS.");
        this->deactivate();
    }
    return 0;
}

void ActsAction::activate()
{
    acts.openPort(myRobot);
    ArAction::activate();

}

void ActsAction::deactivate()
{
    acts.closePort();
    ArAction::deactivate();
}

void ActsAction::printChannel(int channel)
{
    int numBlobs = acts.getNumBlobs(channel);
    if (numBlobs > 0) {
        ArLog::log(ArLog::Normal, "%d blobs in channel %d", numBlobs, channel);
    }
    for (int i = 0; i < numBlobs; ++i) {
        int blobNo = i+1;
        if(acts.getBlob(channel, blobNo, &blob)) {
            ArLog::log(ArLog::Normal, "  Blob %d:",blobNo);
            printBlobInfo(blob);
        } else {
            ArLog::log(ArLog::Normal, "  Can't get blob %d:",blobNo);
        }
    }
}

void ActsAction::printBlobInfo(ArACTSBlob &blob)
{
    ArLog::log(ArLog::Normal, "    Area:        %d",blob.getArea());
    ArLog::log(ArLog::Normal, "    BoundingBox: (%d, %d, %d, %d)",
        blob.getTop(), blob.getLeft(), blob.getBottom(), blob.getRight());
    ArLog::log(ArLog::Normal, "    Position:    (%d, %d)",
        blob.getXCG(), blob.getYCG());
}
