#include <defaultrobotserver.h>

#include <Aria.h>
#include <Arnl.h>

#include "pathtask.h"

int main(int argc, char **argv) {

    DefaultRobotServer server;

    server.init(argc, argv);

    ArRobot *robot = server.getRobot();
    ArMap *map = server.getMap();
    ArPathPlanningTask *pathPlanningTask = server.getPathTask();

    ArLog::log(ArLog::Terse, "Load a map with: -map <MAP_FILE>");

    PathTask pathTask(robot,pathPlanningTask, map);

    robot->addUserTask("PathTask",0,&pathTask);

    server.run();
    return 0;
}
