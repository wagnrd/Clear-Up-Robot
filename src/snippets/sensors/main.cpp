#include <defaultrobotserver.h>

#include "sonarrangeaction.h"
#include "sonarreadingaction.h"
#include "sonarreadingboxaction.h"
#include "laserreadingboxaction.h"

int main(int argc, char **argv) {

    DefaultRobotServer server;

    server.init(argc, argv);

    SonarRangeAction sonarRangeAction;
    SonarReadingAction sonarReadingAction;
    SonarReadingBoxAction sonarReadingBoxAction(server.getSonar());
    LaserReadingBoxAction laserReadingBoxAction(server.getLaser());

    server.addAction(&sonarRangeAction);
    server.addAction(&sonarReadingAction);
    server.addAction(&sonarReadingBoxAction);
    server.addAction(&laserReadingBoxAction);

    sonarRangeAction.activate();
    sonarReadingAction.activate();
    sonarReadingBoxAction.activate();
    laserReadingBoxAction.activate();

    server.run();
    return 0;
}
