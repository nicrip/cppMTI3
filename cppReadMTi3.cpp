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
    mti.runPrintSelfTest();
    // mti.setDefaultOptionFlags();
    mti.printOptionFlags();
    mti.printAvailableFilterProfiles();
    mti.printCurrentFilterProfile();
    // mti.setFilterProfile(51);
    mti.printConfiguration();
    mti.printOutputConfiguration();
    // mti.setDefaultMagneticOutputConfiguration();
    // mti.setDefaultMeasurementOutputConfiguration();

    mti.goToMeasurement();
    while(1) {
        if (mti.readMeasurement()) {
            std::cout << std::dec << mti.getPacketCount() << std::endl;
            std::cout << std::dec << mti.getTemperature() << std::endl;
            for (unsigned int i=0; i<3; i++) {
                std::cout << std::dec << mti.getEulerAngles()[i] << std::endl;
            }
        }
    }
}