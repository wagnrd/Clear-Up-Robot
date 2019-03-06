#include "config.h"
#include <Aria.h>

ConfigSingleton *ConfigSingleton::instance = nullptr;
ConfigSingleton *ConfigSingleton::getInstance()
{
    if (!instance)
    {
        instance = new ConfigSingleton();
    }
    return instance;
}

ConfigSingleton::~ConfigSingleton()
{
    if (acts.isConnected())
    {
        acts.closePort();
    }
}
