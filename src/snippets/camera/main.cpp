#include <defaultrobotserver.h>

#include "cameraaction.h"
#include "actsaction.h"

int main(int argc, char **argv) {

    DefaultRobotServer server;

    server.init(argc, argv);

    CameraAction cameraAction;
    server.addAction(cameraAction);

    ActsAction actsAction;
    server.addAction(actsAction);

    cameraAction.activate();
    actsAction.activate();

    server.run();

    return 0;
}
