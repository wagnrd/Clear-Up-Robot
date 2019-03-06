#ifndef MAPINFOACTION_H
#define MAPINFOACTION_H

#include <ArAction.h>

class ArMap;

class MapInfoAction : public ArAction
{
public:
    MapInfoAction(ArMap *map);
    virtual ArActionDesired *fire(ArActionDesired currentDesired);

protected:
    ArMap *map;
};

#endif // MAPINFOACTION_H
