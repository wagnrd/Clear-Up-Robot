#include "helloWorldAction.h"

HelloWorldAction::HelloWorldAction() :
    ArAction("HelloWorldAction",
             "This action just prints \"Hello World\".")
{
}

ArActionDesired *HelloWorldAction::fire(ArActionDesired currentDesired)
{
    ArLog::log(ArLog::Normal, "Hello World!");
    this->deactivate();
    return 0;
}
