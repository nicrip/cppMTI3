#include "cppMTi3.h"
#include <fstream>

// Script to configure and run the MTi

int main(int argc, char *argv[])
{
    std::string filename;
    unsigned int record_length_ms = 30000;

    if (argc == 2) {
        filename = std::string(argv[1]) + ".mtb";
    } else if (argc == 3) {
        filename = std::string(argv[1]) + ".mtb";
        record_length_ms = (unsigned int)atoi(argv[2]);
    } else {
        filename = "./maglog.mtb";
    }
    std::cout << filename.c_str() << std::endl;

    std::fstream fs(filename.c_str(), std::fstream::trunc | std::fstream::binary | std::fstream::out);

    MTi3 mti = MTi3();
    mti.printDebugMessages(true);
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
    mti.printOutputConfiguration();
    mti.printConfiguration();
    mti.reset();
    bool success;
    success = mti.waitUntilMessage(0x0d, 60000);
    if (success) {
        mti.logAck(&fs);
    } else {
        exit(0);
    }
    success = mti.waitUntilMessage(0x91, 60000);
    if (success) {
        mti.logAck(&fs);
    } else {
        exit(0);
    }
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() < record_length_ms) {
        float elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
        float rem = (record_length_ms/1000.0) - (elapsed/1000.0);
        std::cout << std::dec << "Time Remaining (s): " << rem << std::endl;
        mti.readLogMeasurement(&fs);
    }
}
