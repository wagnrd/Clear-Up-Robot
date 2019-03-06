#include <defaultrobotserver.h>
#include "stateMachine/mainStateMachine.h"

class ArLaser;

int main(int argc, char **argv)
{
    DefaultRobotServer server;
    server.init(argc, argv);

    MainStateMachine mainStateMachine(&server);
    server.addAction(mainStateMachine);
    mainStateMachine.activate();

    server.run();

    return 0;
}
