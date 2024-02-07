#include "cppMTi3.h"

#define XSENS_ADDR 0x6B;

MTi3::MTi3() {
    address = XSENS_ADDR;
    // start PiGPIO object on local computer
    gpio_commander = pigpio_start("192.168.1.5", NULL);
    if (gpio_commander < 0) {
        std::cout << "PiGPIO Failed to Start... Exiting." << std::endl;
        exit(0);
    }
    xbus = new Xbus(gpio_commander, address, 1);
    i2c_handle = getHandle();
}

MTi3::~MTi3() {
    xbus->~Xbus();
    std::cout << "Destructing MTi3 Object..." << std::endl;
}

void MTi3::printDebugMessages(bool set) {
    xbus->print_raw = set;
}

bool MTi3::detect(unsigned int timeout_ms) {
    std::cout << "Detecting MTi3 device..." << std::endl;
    // goToConfig reduced xbus message with extra char for checksum
    char goToConfig[4] = {XSENS_CONTROL_PIPE, xbus->MesID::GOTOCONFIG, '\x00', '\x00'};
    sendMessage(goToConfig, sizeof(goToConfig));
    uint8_t retcode = xbus->readUntilAck(xbus->MesID::GOTOCONFIGACK, timeout_ms);
    if (retcode == xbus->MesID::GOTOCONFIGACK) {
        std::cout << "MTi3 device detected!" << std::endl;
        return true;
    } else if (retcode == 0xFF) {
        std::cout << "Timed out. Failed to detect MTi3 device" << std::endl;
        return false;
    }
    return false;
}

void MTi3::goToConfig() {
    if (!xbus->configState) {
        // goToConfig reduced xbus message with extra char for checksum
        char goToConfig[4] = {XSENS_CONTROL_PIPE, xbus->MesID::GOTOCONFIG, '\x00', '\x00'};
        sendMessage(goToConfig, sizeof(goToConfig));
        uint8_t retcode = xbus->readUntilAck(xbus->MesID::GOTOCONFIGACK, 500);
    }
}

void MTi3::goToMeasurement() {
    if (xbus->configState) {
        // goToMeasurement reduced xbus message with extra char for checksum
        char goToMeasurement[4] = {XSENS_CONTROL_PIPE, xbus->MesID::GOTOMEASUREMENT, '\x00', '\x00'};
        sendMessage(goToMeasurement, sizeof(goToMeasurement));
        uint8_t retcode = xbus->readUntilAck(xbus->MesID::GOTOMEASUREMENTACK, 500);
    }
}

void MTi3::printDeviceInfo() {
    goToConfig();

    // request device ID reduced xbus message with extra char for checksum
    char msg[4] = {XSENS_CONTROL_PIPE, xbus->MesID::REQDID, '\x00', '\x00'};
    sendMessage(msg, sizeof(msg));
    xbus->readUntilAck(xbus->MesID::DEVICEID, 500);

    // request product code reduced xbus message with extra char for checksum
    char msg1[4] = {XSENS_CONTROL_PIPE, xbus->MesID::REQPRODUCTCODE, '\x00', '\x00'};
    sendMessage(msg1, sizeof(msg1));
    xbus->readUntilAck(xbus->MesID::PRODUCTCODE, 500);

    // request hardware version reduced xbus message with extra char for checksum
    char msg2[4] = {XSENS_CONTROL_PIPE, xbus->MesID::REQHARDWAREVERSION, '\x00', '\x00'};
    sendMessage(msg2, sizeof(msg2));
    xbus->readUntilAck(xbus->MesID::HARDWAREVERSION, 500);

    // request firmware revision reduced xbus message with extra char for checksum
    char msg3[4] = {XSENS_CONTROL_PIPE, xbus->MesID::REQFWREV, '\x00', '\x00'};
    sendMessage(msg3, sizeof(msg3));
    xbus->readUntilAck(xbus->MesID::FIRMWAREREV, 500);
}

void MTi3::runPrintSelfTest() {
    goToConfig();

    // request run of self-test reduced xbus message with extra char for checksum
    char runSelfTest[4] = {XSENS_CONTROL_PIPE, xbus->MesID::RUNSELFTEST, '\x00', '\x00'};
    sendMessage(runSelfTest, sizeof(runSelfTest));
    xbus->readUntilAck(xbus->MesID::SELFTESTRESULTS, 5000);
}

void MTi3::setDefaultOptionFlags() {
    goToConfig();

    /* OPTION FLAGS - as default we want the MTI-3: 
        - do not go to measurement mode on startup - set DisableAutoMeasurement (0x00000002)
        - do not enable AHS mode (active heading stabilization), which does not use magnetometer - clear EnableAhs (0x00000010)
        - enable configuration mode on startup - set EnableConfigMessageAtStartup (0x00000200)
       so we have two 8-byte datas, first the SetFlags then the ClearFlags, thus:
       | 0x00 0x00 0x02 0x02 | 0x00 0x00 0x00 0x10 |
    */

    // default option flags reduced xbus message with extra char for checksum
    char setDefOptFlags[12] = {XSENS_CONTROL_PIPE, xbus->MesID::SETOPTIONFLAG, '\x08', /**/ '\x00', '\x00', '\x02', '\x02', 
                                                                                        /**/ '\x00', '\x00', '\x00', '\x10', 
                                                                                        /**/ '\x00'};
    sendMessage(setDefOptFlags, sizeof(setDefOptFlags));
    xbus->readUntilAck(xbus->MesID::OPTIONFLAG, 500);
}

void MTi3::printOptionFlags() {
    goToConfig();

    // request option flags reduced xbus message with extra char for checksum
    char reqOptFlags[4] = {XSENS_CONTROL_PIPE, xbus->MesID::SETOPTIONFLAG, '\x00', '\x00'};
    sendMessage(reqOptFlags, sizeof(reqOptFlags));
    xbus->readUntilAck(xbus->MesID::OPTIONFLAG, 500);
}

void MTi3::setLocationID(uint8_t id_hi, uint8_t id_lo) {
    goToConfig();

    // set location ID (2 bytes) reduced xbus message with extra char for checksum
    char setLocID[6] = {XSENS_CONTROL_PIPE, xbus->MesID::SETLOCATIONID, '\x02', (char)id_hi, (char)id_lo, '\x00'};
    sendMessage(setLocID, sizeof(setLocID));
    xbus->readUntilAck(xbus->MesID::LOCATIONID, 500);
}

void MTi3::printLocationID() {
    goToConfig();

    // request location ID reduced xbus message with extra char for checksum
    char reqLocID[4] = {XSENS_CONTROL_PIPE, xbus->MesID::REQLOCATIONID, '\x00', '\x00'};
    sendMessage(reqLocID, sizeof(reqLocID));
    xbus->readUntilAck(xbus->MesID::LOCATIONID, 500);
}

void MTi3::printAvailableFilterProfiles() {
    goToConfig();

    // request available filter profiles reduced xbus message with extra char for checksum
    char reqAvailFiltProf[4] = {XSENS_CONTROL_PIPE, xbus->MesID::REQAVAILABLEFILTERPROFILES, '\x00', '\x00'};
    sendMessage(reqAvailFiltProf, sizeof(reqAvailFiltProf));
    xbus->readUntilAck(xbus->MesID::AVAILABLEFILTERPROFILES, 500);
}

void MTi3::printCurrentFilterProfile() {
    goToConfig();

    // request current filter profile reduced xbus message with extra char for checksum
    char reqCurrFiltProf[4] = {XSENS_CONTROL_PIPE, xbus->MesID::REQFILTERPROFILE, '\x00', '\x00'};
    sendMessage(reqCurrFiltProf, sizeof(reqCurrFiltProf));
    xbus->readUntilAck(xbus->MesID::REQFILTERPROFILEACK, 500);
}

void MTi3::setFilterProfile(unsigned int profile_id) {
    goToConfig();

    /* FILTER PROFILES - as default on the MTI-3, we want profile 51 (high magnetic dependency):
        - 50 (General): 
            - Standard filter profile with conservative tuning. Several 10s of seconds resistance to magnetic field changes; then slowly converges to the new magnetic field.
            - Can be used for most applications.
        - 51 (High_mag_dep):
            - Filter profile relies heavily on magnetic field. Around 10 seconds resistance to magnetic field changes; then converges to new magnetic field. Assumes a homogeneous magnetic field.
            - Applications that can use the magnetometer and can be magnetic field mapped.
        - 52 (Dynamic):
            - Filter profile assumes fast changes in magnetic field, but also periods where it doesn’t change. Around 10 seconds resistance to magnetic field changes; then quickly converges to new magnetic field.
            - Applications that experience accelerations and dynamic movements, e.g. handheld applications.
        - 53 (North_reference):
            - Filter profiles assumes a homogeneous magnetic field with short magnetic changes allowed. When there is a long-lasting magnetic disturbance, the heading will very slowly converge to the new magnetic field. 
            - Applications that have low magnetic distortions and where fast heading changes are not expected, e.g. Satellite on the Move, buoys.
        - 54 (VRU_general):
            - Behavior as in general for roll and pitch (inclination). The heading is not referenced by the magnetic field. The gyro bias however is estimated continuously, even in in the z-axis. Magnetic distortions may have an effect on the gyro bias estimation accuracy. This filter profile is designed to work most effectively with the Active Heading Stabilization (AHS) feature enabled.
            - Applications where the magnetic field cannot be trusted, e.g. ground robotics in industrial environments.
    */

    // set current filter profile reduced xbus message with extra char for checksum
    char setCurrFiltProf[6] = {XSENS_CONTROL_PIPE, xbus->MesID::SETFILTERPROFILE, '\x02', '\x00', (char)profile_id, '\x00'};
    sendMessage(setCurrFiltProf, sizeof(setCurrFiltProf));
    xbus->readUntilAck(xbus->MesID::REQFILTERPROFILEACK, 500);
}

void MTi3::restoreFactoryDefaults() {
    goToConfig();

    // restore factory defaults reduced xbus message with extra char for checksum
    char restoreFactoryDef[4] = {XSENS_CONTROL_PIPE, xbus->MesID::RESTOREFACTORYDEF, '\x00', '\x00'};
    sendMessage(restoreFactoryDef, sizeof(restoreFactoryDef));
    xbus->readUntilAck(xbus->MesID::WAKEUP, 5000);
}

void MTi3::reset() {
    goToConfig();

    // reset reduced xbus message with extra char for checksum
    char reset[4] = {XSENS_CONTROL_PIPE, xbus->MesID::RESET, '\x00', '\x00'};
    sendMessage(reset, sizeof(reset));
    xbus->readUntilAck(xbus->MesID::WAKEUP, 5000);
}

// sets the default output configuration for data we would like to see during vehicle runtime.
// exits program, since settting the output configuration requires a hard reset.
void MTi3::setDefaultMeasurementOutputConfiguration() {
    goToConfig();

    /* OUTPUT CONFIGURATION - as default for measurements on the MTI-3, we want:
        - Temperature (0x0810) at rate of 1Hz (0x0001) [Temperature must have axis convention of 0]
        - Packet Counter (0x1020) rate is ignored (0xFFFF) [Packet Counter must have axis convention of 0]
        - Sample Time Fine (0x1060) rate is ignored (0xFFFF) [Sample Times must have axis convention of 0]
        - Quaternion in NED (0x2014) at rate of 100Hz (0x0064)
        - Euler Angles in NED (0x2034) at rate of 100Hz (0x0064)
        - Accelerations in NED (0x4024) at rate of 100Hz (0x0064)
        - Rates of Turn (0x8020) at rate of 100Hz (0x0064) [Gyro must have axis convention of 0]
        - Magnetic Field (0xC020) at rate of 100Hz (0x0064) [Magnetometer must have axis convention of 0]
        - Status Word (0xE020)  rate is ignored (0xFFFF) [Statuses must have axis convention of 0]
    */

   // SetOutputConfiguration reduced xbus message with char for checksum
   char outputConfig[40] = {XSENS_CONTROL_PIPE, xbus->MesID::SETOUTPUTCONFIGURATION, '\x24', /**/ '\x08', '\x10', '\x00', '\x01', 
                                                                                            /**/ '\x10', '\x20', '\xFF', '\xFF', 
                                                                                            /**/ '\x10', '\x60', '\xFF', '\xFF', 
                                                                                            /**/ '\x20', '\x14', '\x00', '\x64', 
                                                                                            /**/ '\x20', '\x34', '\x00', '\x64', 
                                                                                            /**/ '\x40', '\x24', '\x00', '\x64', 
                                                                                            /**/ '\x80', '\x20', '\x00', '\x64', 
                                                                                            /**/ '\xC0', '\x20', '\x00', '\x64', 
                                                                                            /**/ '\xE0', '\x20', '\xFF', '\xFF', 
                                                                                            /**/ '\x00'};
    sendMessage(outputConfig, sizeof(outputConfig));
    exit(0);
}

// sets the default output configuration for data we would like to see during magnetic calibration logging.
// exits program, since settting the output configuration requires a hard reset.
void MTi3::setDefaultMagneticOutputConfiguration() {
    goToConfig();

    /* OUTPUT CONFIGURATION - as default for magnetic calibration logging on the MTI-3, we want:
        - Packet Counter (0x1020) rate is ignored (0xFFFF) [Packet Counter must have axis convention of 0]
        - Sample Time Fine (0x1060) rate is ignored (0xFFFF) [Sample Times must have axis convention of 0]
        - Delta V in ENU (0x4010) at rate of 100Hz (0x0064)
        - Delta Q (0x8030) at rate of 100Hz (0x0064) [Gyro must have axis convention of 0]
        - Magnetic Field (0xC020) at rate of 100Hz (0x0064) [Magnetometer must have axis convention of 0]
    */

   // SetOutputConfiguration reduced xbus message with char for checksum
   char outputConfig[24] = {XSENS_CONTROL_PIPE, xbus->MesID::SETOUTPUTCONFIGURATION, '\x14', /**/ '\x10', '\x20', '\xFF', '\xFF', 
                                                                                            /**/ '\x10', '\x60', '\xFF', '\xFF', 
                                                                                            /**/ '\x40', '\x10', '\x00', '\x64', 
                                                                                            /**/ '\x80', '\x30', '\x00', '\x64', 
                                                                                            /**/ '\xC0', '\x20', '\x00', '\x64', 
                                                                                            /**/ '\x00'};
    sendMessage(outputConfig, sizeof(outputConfig));
    exit(0);
}

void MTi3::printOutputConfiguration() {
    goToConfig();

    // reset reduced xbus message with extra char for checksum
    char reqOutputConfig[4] = {XSENS_CONTROL_PIPE, xbus->MesID::REQOUTPUTCONFIGURATION, '\x00', '\x00'};
    sendMessage(reqOutputConfig, sizeof(reqOutputConfig));
    xbus->readUntilAck(xbus->MesID::OUTPUTCONFIGURATION, 500);
}

void MTi3::sendMessage(char *message, uint8_t numBytes) {
    xbus->sendMessage(message, numBytes);
}

bool MTi3::readMeasurement() {
    bool new_measurement = xbus->read();
    return(new_measurement);
}

bool MTi3::readLogMeasurement(std::fstream* fs) {
    bool new_measurement = xbus->read();
    if (new_measurement) {
        fs->write(xbus->datameas, sizeof(xbus->datameas));
    }
    return(new_measurement);
}

void MTi3::logAck(std::fstream* fs) {
    fs->write(xbus->datanotif, sizeof(xbus->datanotif));
}