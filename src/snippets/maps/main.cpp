#include <defaultrobotserver.h>

#include <Aria.h>

#include "mapinfoaction.h"

int main(int argc, char **argv) {

    DefaultRobotServer server;

    server.init(argc, argv);

    ArMap *map = server.getMap();

    ArLog::log(ArLog::Terse, "Load a map with: -map <MAP_FILE>");
    MapInfoAction mapInfo(map);
    server.addAction(&mapInfo, 50);

    mapInfo.activate();

    server.run();
    return 0;
}
