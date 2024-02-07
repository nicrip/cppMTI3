#include "cppMTi3.h"

// Script to configure and run the MTi

int main()
{
    MTi3 mti = MTi3();
    mti.printDebugMessages(false);
    bool detect = mti.detect(1000);
    if (!detect) exit(0);
    mti.goToConfig();
    mti.printDeviceInfo();
    mti.printOptionFlags();
    mti.printAvailableFilterProfiles();
    mti.printCurrentFilterProfile();
    mti.printOutputConfiguration();
}