#include "mapinfoaction.h"

#include <vector>
#include <list>
#include <iterator>

#include <Aria.h>

MapInfoAction::MapInfoAction(ArMap *map) :
    ArAction("MapInfoAction",
             "This action prints some Map informations."),
    map(map)
{
}

ArActionDesired *MapInfoAction::fire(ArActionDesired currentDesired)
{
    if (!map) {
        ArLog::log(ArLog::Terse, "Warning a map is needed for MapInfoAction!");
        deactivate();
        return 0;
    }

    ArLog::log(ArLog::Normal, "Loaded map: %s", map->getFileName());

    ArLog::log(ArLog::Normal, "Lines: %d", map->getNumLines());
    std::vector<ArLineSegment> *lines = map->getLines();
    for (std::size_t i = 0; i < lines->size(); ++i) {
        ArLineSegment line = lines->at(i);
        ArLog::log(ArLog::Normal, "%d: (%5.0f, %5.0f, %5.0f, %5.0f)",
                   i, line.getX1(), line.getY1(), line.getX2(),
                   line.getY2());
    }

    std::list<ArMapObject *> homePoints =
            map->findMapObjectsOfType("RobotHome", true);
    ArLog::log(ArLog::Normal, "Home points: %d", homePoints.size());
    for (std::list<ArMapObject *>::iterator it = homePoints.begin();
         it != homePoints.end(); it++){
        ArMapObject *mapObject = *it;
        ArPose pose = mapObject->getPose();
        ArLog::log(ArLog::Normal, "(%5.0f, %5.0f, %3.2f)",
                   pose.getX(), pose.getY(), pose.getTh());
    }

    std::list<ArMapObject *> goals =
            map->findMapObjectsOfType("Goal", true);
    ArLog::log(ArLog::Normal, "Goals: %d", goals.size());
    for (std::list<ArMapObject *>::iterator it = goals.begin();
         it != goals.end(); it++){
        ArMapObject *mapObject = *it;
        ArPose pose = mapObject->getPose();
        ArLog::log(ArLog::Normal, "(%5.0f, %5.0f, %3.2f)",
                   pose.getX(), pose.getY(), pose.getTh());
    }

    return 0;
}
