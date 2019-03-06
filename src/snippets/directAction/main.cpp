#include <Aria.h>
#include <defaultrobotserver.h>

#include "patrolaction.h"

void print(const char *msg) {
    ArLog::log(ArLog::Normal, msg);
}

int main(int argc, char **argv) {

    DefaultRobotServer server;
    server.init(argc, argv);

    PatrolAction patrol;
    server.addAction(patrol);
    patrol.activate();

    server.run();

    Aria::exit(0);
    return 0;
}
