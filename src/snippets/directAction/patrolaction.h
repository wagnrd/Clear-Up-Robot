#ifndef PATROLACTION_H
#define PATROLACTION_H

#include <ArAction.h>

class PatrolAction : public ArAction
{
public:
    PatrolAction();
    virtual ArActionDesired *fire(ArActionDesired currentDesired);

    enum State {
      Move,
      Turn
    };

protected:
    State state;

};

#endif // PATROLACTION_H
