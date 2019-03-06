#include <ArActionGoto.h>
#include <defaultrobotserver.h>

#include "robotinfoaction.h"

int main(int argc, char **argv) {

    DefaultRobotServer server;

    server.init(argc, argv);
    ArPose goal(1000,1000,0);
    ArActionGoto gotoAction("goto", goal);
    RobotInfoAction robotInfoAction;

    server.addAction(&gotoAction, 51);
    server.addAction(&robotInfoAction,50);

    gotoAction.activate();
    robotInfoAction.activate();

    server.run();

    return 0;
}
