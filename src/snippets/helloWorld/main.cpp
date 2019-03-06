#include <defaultrobotserver.h>
#include "helloWorldAction.h"

int main(int argc, char **argv) {

    DefaultRobotServer server;
    server.init(argc, argv);

    HelloWorldAction helloWorldAction;
    server.addAction(helloWorldAction);
    helloWorldAction.activate();

    server.run();
    return 0;
}
