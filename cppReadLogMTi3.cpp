#include "cppMTi3.h"
#include <fstream>

// Script to configure and run the MTi

int main(int argc, char *argv[])
{
    std::string filename;
    unsigned int record_length_ms = 30000;
    unsigned int stabilize_length_ms = 5000;

    if (argc == 2) {
        filename = std::string(argv[1]) + ".mtb";
    } else if (argc == 3) {
        filename = std::string(argv[1]) + ".mtb";
        record_length_ms = (unsigned int)atoi(argv[2]);
    } else {
        filename = "./maglog.mtb";
    }
    std::cout << filename.c_str() << std::endl;

    MTi3 mti = MTi3();
    mti.printDebugMessages(false);
    bool detect = mti.detect(1000);
    if (!detect) exit(0);
    std::fstream fs(filename.c_str(), std::fstream::trunc | std::fstream::binary | std::fstream::out);
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
    mti.logAck(&fs);
    mti.printeMTS();
    mti.logAck(&fs);
    mti.goToMeasurement();
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() < stabilize_length_ms) {
        float elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
        std::cout << std::dec << "Stabilizing..." << std::endl;
        mti.readMeasurement();
    }
    start = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() < record_length_ms) {
        float elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
        float rem = (record_length_ms/1000.0) - (elapsed/1000.0);
        std::cout << std::dec << "Time Remaining (s): " << rem << std::endl;
        mti.readLogMeasurement(&fs);
    }
    fs.close();
}
