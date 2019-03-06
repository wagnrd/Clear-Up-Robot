#include <ArAction.h>

class HelloWorldAction : public ArAction
{
public:
    HelloWorldAction();
    ArActionDesired *fire(ArActionDesired currentDesired);
};
