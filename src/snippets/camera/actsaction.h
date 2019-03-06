#ifndef ACTSACTION_H
#define ACTSACTION_H

#include <ArAction.h>
#include <ArACTS.h>

class ActsAction : public ArAction
{
public:
    ActsAction();
    ~ActsAction();
    ArActionDesired *fire(ArActionDesired currentDesired);

    virtual void activate();
    virtual void deactivate();

    void printChannel(int channel);
    void printBlobInfo(ArACTSBlob &blob);

protected:
    ArACTS_1_2 acts;
    ArACTSBlob blob;
};

#endif // ACTSACTION_H
