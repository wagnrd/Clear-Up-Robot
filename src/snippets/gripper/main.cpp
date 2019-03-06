#include <defaultrobotserver.h>

#include "gripperaction.h"

int main(int argc, char **argv) {

    DefaultRobotServer server;

    server.init(argc, argv);

    GripperAction gripperAction(server.getGripper());
    server.addAction(&gripperAction,50);

    gripperAction.activate();

    server.run();

    return 0;
}
