#include "Xbus.h"

Xbus::Xbus() {
    
}

Xbus::Xbus(int pigpio_id, uint8_t I2C_ADDR, uint8_t I2C_BUS) {
    gpio_commander = pigpio_id;
    I2C_ADDR_ = I2C_ADDR;
    I2C_BUS_ = I2C_BUS;
    // Open i2c port
    i2c_handle = i2c_open(gpio_commander, I2C_BUS, I2C_ADDR, 0);
    if (i2c_handle < 0) {
        std::cout << "Can't open I2C Bus" << std::endl;
        exit(0);
    } else {
        std::cout << "I2C Bus Opened" << std::endl;
    }
    print_raw = false;
}

Xbus::~Xbus() {
    std::cout << "Closing XBus I2C Line..." << std::endl;
    i2c_close(gpio_commander, i2c_handle);
}

bool Xbus::read() {
    readPipeStatus();
    if (notificationSize > 0) { // New notification message available to be read
        readPipeNotif();
        parseNotification(datanotif, notificationSize);
    }
    if (measurementSize > 0) { // New measurement packet available to be read
        readPipeMeas();
        parseMTData2(datameas, measurementSize);
        // new measurement read
        return(true);
    } else {
        // no new measurement
        return(false);
    }
}

uint8_t Xbus::readUntilAck(uint8_t opcode, int timeout) {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() < timeout) {
        uint8_t retcode;
        readPipeStatus();
        if (notificationSize > 0) { // New notification message available to be read
            readPipeNotif();
            if (notificationSize <= 256) retcode = parseNotification(datanotif, notificationSize);
        }
        if (measurementSize > 0) { // New measurement packet available to be read
            readPipeMeas();
            parseMTData2(datameas, measurementSize);
        } else {
        }
        if (retcode == opcode) {
            // we got the correct opcode for acknowledgement
            return retcode;
        }
    }
    // timed out - acknowledgement not received
    return 0xFF;
}

void Xbus::readPipeStatus() {
    // std::cout << "reading status" << std::endl;
    i2c_write_byte(gpio_commander, i2c_handle, XSENS_STATUS_PIPE);
    i2c_read_i2c_block_data(gpio_commander, i2c_handle, XSENS_STATUS_PIPE, status, 4);
    notificationSize = (uint16_t)status[0] | ((uint16_t)status[1] << 8);
    measurementSize = (uint16_t)status[2] | ((uint16_t)status[3] << 8);

    // Uncomment to see notif and meas size
    if (print_raw) {
        if (notificationSize || measurementSize) {
            std::cout << "Notification Size: " << std::dec << int(notificationSize) << " | ";
            std::cout << "Measurement Size: " << std::dec << int(measurementSize) << std::endl;
        }
    }
}

void Xbus::readPipeNotif() {
    i2c_write_byte(gpio_commander, i2c_handle, XSENS_NOTIF_PIPE);
    if (notificationSize <= 256) {
        i2c_read_device(gpio_commander, i2c_handle, datanotif, notificationSize);
    } else {
        std::cout << "Invalid Notification Msg of Size " << (int)notificationSize << std::endl;
        char temp_notif[notificationSize];
        i2c_read_device(gpio_commander, i2c_handle, temp_notif, notificationSize);
    }
    
    // Uncomment to see hex values of notification data
    if (print_raw) {
        if (notificationSize <= 256) {
            uint8_t *datanotif_2;
            datanotif_2 = (uint8_t *)datanotif;
            std::cout << "Notification Msg (Hex): ";
            for (int i = 0; i < notificationSize; i++) {
                std::cout << "0x" << std::setfill('0') << std::setw(2) << std::right << std::hex << (int)datanotif_2[i];
                if (i < notificationSize-1) {
                    std::cout << ":";
                }
            }
            std::cout << std::endl;
        }
    }
}

void Xbus::readPipeMeas() {
    i2c_write_byte(gpio_commander, i2c_handle, XSENS_MEAS_PIPE);
    if (measurementSize <= 256) {
        i2c_read_device(gpio_commander, i2c_handle, datameas, measurementSize);
    } else {
        std::cout << "Invalid Measurement Msg of Size " << (int)measurementSize << std::endl;
        char temp_meas[measurementSize];
        i2c_read_device(gpio_commander, i2c_handle, temp_meas, measurementSize);
    }
    
    // Uncomment to see hex values of measurement data
    if (print_raw) {
        if (measurementSize <= 256) {
            uint8_t *datameas_2;
            datameas_2 = (uint8_t *)datameas;
            std::cout << "Measurement Msg (Hex): ";
            for (int i = 0; i < measurementSize; i++) {
                std::cout << "0x" << std::setfill('0') << std::setw(2) << std::right << std::hex << (int)datameas_2[i];
                if (i < measurementSize-1) {
                    std::cout << ":";
                }
            }
            std::cout << std::endl;
        }
    }
}

// Parse the most common notification messages
uint8_t Xbus::parseNotification(char *notif, uint8_t notiflength) {
    uint8_t notifID = notif[0];
    switch (notifID) {
        
        // Wake Up + State Messages
        case (uint8_t)MesID::WAKEUP: {
            std::cout << "--- Received WakeUp Message (0x3E). ---" << std::endl;
            // move to config state on wakeup
            char wake_ack[4] = {XSENS_CONTROL_PIPE, MesID::WAKEUPACK, '\x00', '\x00'};
            sendMessage(wake_ack, sizeof(wake_ack));
            std::cout << "    Sent WakeUp Acknowledgement" << std::endl;
            wakeState = true;
            return (WAKEUP);
        } case (uint8_t)MesID::GOTOCONFIGACK: {
            std::cout << "--- Received GoToConfig Acknowledgement (0x31). ---" << std::endl;
            configState = true;
            return (GOTOCONFIGACK);
        } case (uint8_t)MesID::GOTOMEASUREMENTACK: {
            std::cout << "--- Received GoToMeasurement Acknowledgement (0x11). ---" << std::endl;
            configState = false;
            return (GOTOMEASUREMENTACK);
        } case (uint8_t)MesID::RESETACK: {
            std::cout << "--- Received Reset Acknowledgement (0x41). ---" << std::endl;
            return (RESETACK);
        
        // Informational Messages
        } case (uint8_t)MesID::DEVICEID: {
            std::cout << "--- Received Device ID (0x01): ";
            uint32_t deviceID = (uint8_t)notif[2] << 24 | (uint8_t)notif[3] << 16 | (uint8_t)notif[4] << 8 | (uint8_t)notif[5];
            std::cout << std::dec <<  (int)deviceID << " ---" << std::endl;
            return(DEVICEID);
        } case (uint8_t)MesID::PRODUCTCODE: {
            std::cout << "--- Received Product Code (0x1D): ";
            for (int i = 2; i < notiflength - 1; i++) {
                std::cout << char(notif[i]);
            }
            std::cout << " ---" << std::endl;
            productCode = notif[6]; // Store the product code (MTi-#) for later usage
            return (PRODUCTCODE);
        } case (uint8_t)MesID::HARDWAREVERSION: {
            std::cout << "--- Received Hardware Version (0x1F): ";
            std::cout << std::dec << (int)notif[2] << "." << (int)notif[3] << " ---" << std::endl;
            return(HARDWAREVERSION);
        } case (uint8_t)MesID::FIRMWAREREV: {
            std::cout << "--- Received Firmware Revision (0x13): ";
            std::cout << std::dec << int(notif[2]) << "." << int(notif[3]) << "." << int(notif[4]) << " ---" << std::endl;
            return (FIRMWAREREV);
        } case (uint8_t)MesID::SELFTESTRESULTS: {
            std::cout << "--- Received Self Test Results (0x25): ";
            uint16_t self_test_results = (uint8_t)notif[2] << 8 | (uint8_t)notif[3];
            std::bitset<16> bit_results(self_test_results);
            std::cout << bit_results << " ---" << std::endl;
            parseSelfTestResults(bit_results);
            return (SELFTESTRESULTS);
        } case (uint8_t)MesID::ERROR: {
            std::cout << "--- Received Error (0x42): ";
            std::cout << "0x" << std::setfill('0') << std::setw(2) << std::right << std::hex << int(notif[2]) << " ---" << std::endl;
            parseError(int(notif[2]));
            return (ERROR);
        } case (uint8_t)MesID::WARNING: {
            std::cout << "--- Received Warning (0x43): ";
            uint32_t warn = (uint32_t)notif[5] | ((uint32_t)notif[4] << 8);
            std::cout << "0x" << std::setfill('0') << std::setw(2) << std::right << std::hex << warn << " ---" << std::endl;
            return (WARNING);
        
        // Device-Specific Messages
        } case (uint8_t)MesID::OPTIONFLAG: {
            std::cout << "--- Received ReqOptionFlag Acknowledgement (0x49). Option flags: " << "0x" << std::setfill('0');
            for (uint64_t i = 2; i < notiflength-1; i++) {
                std::cout << std::setw(2) << std::right << std::hex << (int)notif[i];
            }
            std::cout << " ---" << std::endl;
            uint32_t optionFlag = (uint8_t)notif[2] << 24 | (uint8_t)notif[3] << 16 | (uint8_t)notif[4] << 8 | (uint8_t)notif[5];
            parseOptionFlag(optionFlag);
            return (OPTIONFLAG);
        } case (uint8_t)MesID::LOCATIONID: {
            std::cout << "--- Received ReqLocationID Acknowledgement (0x85). Location ID: " << "0x" << std::setfill('0');
            for (uint64_t i = 2; i < notiflength-1; i++) {
                std::cout << std::setw(2) << std::right << std::hex << (int)notif[i];
            }
            std::cout << " ---" << std::endl;
            return (LOCATIONID);
        } case (uint8_t)MesID::RESTOREFACTORYDEFACK: {
            std::cout << "--- Received RestoreFactoryDefaults Acknowledgement (0x0F). ---" << std::endl;
            return (RESTOREFACTORYDEFACK);

        // Configuration Messages
        } case (uint8_t)MesID::CONFIGURATION: {
            std::cout << "--- Received ReqConfiguration Acknowledgement (0x0D). Configuration: ---" << std::endl;
            std::cout << "  Master device ID: " << std::dec << ((uint8_t)notif[2] << 24 | (uint8_t)notif[3] << 16 | (uint8_t)notif[4] << 8 | (uint8_t)notif[5]) << std::endl;
            std::cout << "  Sampling period: " << std::dec << ((uint8_t)notif[6] << 8 | (uint8_t)notif[7]) << std::endl;
            std::cout << "  Output skip factor: " << std::dec << ((uint8_t)notif[8] << 8 | (uint8_t)notif[9]) << std::endl;
            std::cout << "  SyncIn settings - Mode: " << std::dec << ((uint8_t)notif[10] << 8 | (uint8_t)notif[11]) << std::endl;
            std::cout << "  SyncIn settings - Skip Factor: " << std::dec << ((uint8_t)notif[12] << 8 | (uint8_t)notif[13]) << std::endl;
            std::cout << "  SyncIn settings - Offset: " << std::dec << ((uint8_t)notif[14] << 24 | (uint8_t)notif[15] << 16 | (uint8_t)notif[16] << 8 | (uint8_t)notif[17]) << std::endl;
            std::cout << "  Number of devices: " << std::dec << ((uint8_t)notif[98] << 8 | (uint8_t)notif[99]) << std::endl;
            std::cout << "  Device ID: " << std::dec << ((uint8_t)notif[100] << 24 | (uint8_t)notif[101] << 16 | (uint8_t)notif[102] << 8 | (uint8_t)notif[103]) << std::endl;
            std::cout << "  Data length of MTData2 message: " << std::dec << ((uint8_t)notif[104] << 8 | (uint8_t)notif[105]) << std::endl;
            std::cout << "  Output mode: " << std::dec << ((uint8_t)notif[106] << 8 | (uint8_t)notif[107]) << std::endl;
            std::cout << "  Output settings: " << std::dec << ((uint8_t)notif[108] << 24 | (uint8_t)notif[109] << 16 | (uint8_t)notif[110] << 8 | (uint8_t)notif[111]) << std::endl;
            return (CONFIGURATION);
        } case (uint8_t)MesID::OUTPUTCONFIGURATION: {
            std::cout << "--- Received ReqOutputConfiguration Acknowledgement (0xC1). Output Configuration: ---" << std::endl;
            unsigned int i = 2;
            while (i < notiflength-1) {
                uint16_t gtf = (uint16_t)notif[i] << 8 | (uint16_t)notif[i+1];
                std::cout << "  group, type, format: " << "0x" << std::setfill('0') << std::setw(2) << std::right << std::hex << gtf;
                std::cout << " | frequency: " << std::dec << ((uint16_t)notif[i+2] << 8 | (uint16_t)notif[i+3]) << " Hz" << std::endl;
                parseDataIdentifier(gtf);
                i += 4;
            }
            return (OUTPUTCONFIGURATION);
        } case (uint8_t)MesID::ICCCOMANDACK: {
            std::cout << "--- Received ICCCommand Acknowledgement (0x75). ---" << std::endl;
            for (uint64_t i = 0; i < sizeof(notif); i++) {
                std::cout << std::hex << int(notif[2]) << " ";
            }
            std::cout << std::endl;
            return (ICCCOMANDACK);
        
        // Filter Messages
        } case (uint8_t)MesID::AVAILABLEFILTERPROFILES: {
            std::cout << "--- Received ReqAvailableFilterProfiles Acknowledement (0x63). Available Filter Profiles: ---" << std::endl;
            unsigned int idx;
            for (unsigned int i=0; i<5; i++) {
                std::cout << "  profile " << (int)i+1 << ": " << std::endl;
                std::cout << "    ";
                for (unsigned int j=0; j<22; j++) {
                    idx = 2+i*22+j;
                    if (j == 0) std::cout << std::dec << "num: " << (int)((uint8_t)notif[idx]) << " | ";
                    if (j == 1) std::cout << std::dec << "ver: " << (int)((uint8_t)notif[idx]) << " | ";
                    if (j == 2) std::cout << "desc: ";
                    if (j > 1) {
                        if ((uint8_t)notif[idx] == 0x20) {
                            std::cout << std::endl;
                            break;
                        } else {
                            std::cout << (char)((uint8_t)notif[idx]);
                        } 
                    }
                }
            }
            return(AVAILABLEFILTERPROFILES);
        } case (uint8_t)MesID::REQFILTERPROFILEACK: {
            std::cout << "--- Received ReqFilterProfile Acknowledement (0x65). Current Filter Profile: ---" << std::endl;
            uint8_t profile = (uint8_t)notif[3];
            std::cout << std::dec << "  ver: " << (int)((uint8_t)notif[2]) << " | num: " << (int)profile << " | desc: ";
            if (profile == 50) std::cout << "general" << std::endl;
            if (profile == 51) std::cout << "high_mag_dep" << std::endl;
            if (profile == 52) std::cout << "dynamic" << std::endl;
            if (profile == 53) std::cout << "north_reference" << std::endl;
            if (profile == 54) std::cout << "vru_general" << std::endl;
            return(REQFILTERPROFILEACK);
          
        // Default
        } default: {
            std::cout << "--- Received undefined notification: ---" << std::endl;
            return (0xFF);
        }
    }
}

void Xbus::parseDataIdentifier(uint16_t gtf) {
    if ((gtf & 0x0ff0) == (0x0810)) std::cout << "      --> Temperature Celsius set to output" << std::endl;
    if ((gtf & 0xf0f0) == (0x1010)) std::cout << "      --> UTC Time set to output" << std::endl;
    if ((gtf & 0xf0f0) == (0x1020)) std::cout << "      --> Packet Counter set to output" << std::endl;
    if ((gtf & 0xf0f0) == (0x1060)) std::cout << "      --> Sample Time Fine set to output" << std::endl;
    if ((gtf & 0xf0f0) == (0x1070)) std::cout << "      --> Sample Time Coarse set to output" << std::endl;
    if ((gtf & 0xf0f0) == (0x2010)) std::cout << "      --> Quaternion Data set to output" << std::endl;
    if ((gtf & 0xf0f0) == (0x2020)) std::cout << "      --> Rotation Matrix Data set to output" << std::endl;
    if ((gtf & 0xf0f0) == (0x2030)) std::cout << "      --> Euler Data set to output" << std::endl;
    if ((gtf & 0xf0f0) == (0x3010)) std::cout << "      --> Pressure Pascals set to output" << std::endl;
    if ((gtf & 0xf0f0) == (0x4010)) std::cout << "      --> Delta V set to output" << std::endl;
    if ((gtf & 0xf0f0) == (0x4020)) std::cout << "      --> Acceleration set to output" << std::endl;
    if ((gtf & 0xf0f0) == (0x4030)) std::cout << "      --> Free Acceleration set to output" << std::endl;
    if ((gtf & 0xf0f0) == (0x4040)) std::cout << "      --> Acceleration HR set to output" << std::endl;
    if ((gtf & 0xf0f0) == (0x5020)) std::cout << "      --> Altitude Ellipsoid set to output" << std::endl;
    if ((gtf & 0xf0f0) == (0x5030)) std::cout << "      --> Position ECEF set to output" << std::endl;
    if ((gtf & 0xf0f0) == (0x5040)) std::cout << "      --> Lat/Lon set to output" << std::endl;
    if ((gtf & 0xf0f0) == (0x7010)) std::cout << "      --> GNSS PVT Data set to output" << std::endl;
    if ((gtf & 0xf0f0) == (0x7020)) std::cout << "      --> GNSS Satellite Info set to output" << std::endl;
    if ((gtf & 0xf0f0) == (0x7030)) std::cout << "      --> GNSS PVT Pulse set to output" << std::endl;
    if ((gtf & 0xf0f0) == (0x8020)) std::cout << "      --> Rate of Turn set to output" << std::endl;
    if ((gtf & 0xf0f0) == (0x8030)) std::cout << "      --> Delta Q set to output" << std::endl;
    if ((gtf & 0xf0f0) == (0x8040)) std::cout << "      --> Rate of Turn HR set to output" << std::endl;
    if ((gtf & 0xf0f0) == (0xA010)) std::cout << "      --> ACC, GYR, MAG, Temperature set to output" << std::endl;
    if ((gtf & 0xf0f0) == (0xA020)) std::cout << "      --> Gyro Temperatures set to output" << std::endl;
    if ((gtf & 0xf0f0) == (0xC020)) std::cout << "      --> Magnetic Field set to output" << std::endl;
    if ((gtf & 0xf0f0) == (0xD010)) std::cout << "      --> Velocity XYZ set to output" << std::endl;
    if ((gtf & 0xf0f0) == (0xE010)) std::cout << "      --> Status Byte set to output" << std::endl;
    if ((gtf & 0xf0f0) == (0xE020)) std::cout << "      --> Status Word set to output" << std::endl;
    if ((gtf & 0xf0f0) == (0xE080)) std::cout << "      --> Device ID set to output" << std::endl;
    if ((gtf & 0xf0f0) == (0xE090)) std::cout << "      --> Location ID set to output" << std::endl;
    if ((gtf & 0x000f) == (0x0000)) std::cout << "      --> East-North-Up set to output" << std::endl;
    if ((gtf & 0x000f) == (0x0004)) std::cout << "      --> North-East-Down set to output" << std::endl;
    if ((gtf & 0x000f) == (0x0008)) std::cout << "      --> North-West-Up set to output" << std::endl;
}

void Xbus::parseOptionFlag(uint32_t option_flag) {
    if ((option_flag & 0x0000000f) == (0x00000001)) std::cout << " --> DisableAutoStore set" << std::endl; else std::cout<< " -x- DisableAutoStore not set" << std::endl;
    if ((option_flag & 0x0000000f) == (0x00000002)) std::cout << " --> DisableAutoMeasurement set" << std::endl; else std::cout<< " -x- DisableAutoMeasurement not set" << std::endl;
    if ((option_flag & 0x0000000f) == (0x00000004)) std::cout << " --> EnableBeidou set" << std::endl; else std::cout<< " -x- EnableBeidou not set" << std::endl;
    if ((option_flag & 0x000000f0) == (0x00000010)) std::cout << " --> EnableAhs set" << std::endl; else std::cout<< " -x- EnableAhs not set" << std::endl;
    if ((option_flag & 0x000000f0) == (0x00000020)) std::cout << " --> EnableOrientationSmoother set" << std::endl; else std::cout<< " -x- EnableOrientationSmoother not set" << std::endl;
    if ((option_flag & 0x000000f0) == (0x00000040)) std::cout << " --> EnableConfigurableBusId set" << std::endl; else std::cout<< " -x- EnableConfigurableBusId not set" << std::endl;
    if ((option_flag & 0x000000f0) == (0x00000080)) std::cout << " --> EnableInRunCompassCalibration set" << std::endl; else std::cout<< " -x- EnableInRunCompassCalibration not set" << std::endl;
    if ((option_flag & 0x00000f00) == (0x00000200)) std::cout << " --> EnableConfigMessageAtStartup set" << std::endl; else std::cout<< " -x- EnableConfigMessageAtStartup not set" << std::endl;
    if ((option_flag & 0x00000f00) == (0x00000800)) std::cout << " --> EnablePositionVelocitySmoother set" << std::endl; else std::cout<< " -x- EnablePositionVelocitySmoother not set" << std::endl;
    if ((option_flag & 0x0000f000) == (0x00001000)) std::cout << " --> EnableContinuousZRU set" << std::endl; else std::cout<< " -x- EnableContinuousZRU not set" << std::endl;
}

void Xbus::parseSelfTestResults(std::bitset<16> bit_results) {
    if (bit_results.test(0) == 1) std::cout << " --> accX passed self test" << std::endl; else std::cout << " -x- accX failed self test" << std::endl;
    if (bit_results.test(1) == 1) std::cout << " --> accY passed self test" << std::endl; else std::cout << " -x- accY failed self test" << std::endl;
    if (bit_results.test(2) == 1) std::cout << " --> accZ passed self test" << std::endl; else std::cout << " -x- accZ failed self test" << std::endl;
    if (bit_results.test(3) == 1) std::cout << " --> gyrX passed self test" << std::endl; else std::cout << " -x- gyrX failed self test" << std::endl;
    if (bit_results.test(4) == 1) std::cout << " --> gyrY passed self test" << std::endl; else std::cout << " -x- gyrY failed self test" << std::endl;
    if (bit_results.test(5) == 1) std::cout << " --> gyrZ passed self test" << std::endl; else std::cout << " -x- gyrZ failed self test" << std::endl;
    if (bit_results.test(6) == 1) std::cout << " --> magX passed self test" << std::endl; else std::cout << " -x- magX failed self test" << std::endl;
    if (bit_results.test(7) == 1) std::cout << " --> magY passed self test" << std::endl; else std::cout << " -x- magY failed self test" << std::endl;
    if (bit_results.test(8) == 1) std::cout << " --> magZ passed self test" << std::endl; else std::cout << " -x- magZ failed self test" << std::endl;
    if (bit_results.test(9) == 1) std::cout << " --> baro passed self test" << std::endl; else std::cout << " -x- baro failed self test" << std::endl;
    if (bit_results.test(10) == 1) std::cout << " --> GNSS passed self test" << std::endl; else std::cout << " -x- GNSS failed self test" << std::endl;
    if (bit_results.test(11) == 1) std::cout << " --> batt passed self test" << std::endl; else std::cout << " -x- batt failed self test" << std::endl;
    if (bit_results.test(12) == 1) std::cout << " --> flash passed self test" << std::endl; else std::cout << " -x- flash failed self test" << std::endl;
    if (bit_results.test(13) == 1) std::cout << " --> button passed self test" << std::endl; else std::cout << " -x- button failed self test" << std::endl;
    if (bit_results.test(14) == 1) std::cout << " --> sync passed self test" << std::endl; else std::cout << " -x- sync failed self test" << std::endl;
}

void Xbus::parseError(int error_code) {
    if (error_code == 0x03) std::cout << "  Error: Period sent is not within valid range" << std::endl;
    if (error_code == 0x04) std::cout << "  Error: Message sent is invalid" << std::endl;
    if (error_code == 0x1E) std::cout << "  Error: Timer overflow. This can be caused by a too high output frequency or sending too much data to the MT during measurement" << std::endl;
    if (error_code == 0x20) std::cout << "  Error: Baud rate requested is not within valid range" << std::endl;
    if (error_code == 0x21) std::cout << "  Error: Parameter sent is invalid or not within range" << std::endl;
    if (error_code == 0x28) std::cout << "  Error: Device Error - try updating the firmware; extra device error contains 5 bytes" << std::endl;
    if (error_code == 0x29) std::cout << "  Error: Data overflow, the device generates more data than the bus communication can handle (baud rate may be too low)" << std::endl;
    if (error_code == 0x2A) std::cout << "  Error: Buffer overflow, the sample buffer of the device was full during a communication outage" << std::endl;
}

// recursive data parser
void Xbus::parseMTData2(char *data, uint8_t datalength) {
    if (datalength < 2) return;             // Reached the end of the MTData2 message

    uint8_t XDI = data[0];                  // Xsens Data Identifier
    if (XDI == (uint8_t)MesID::MTDATA2) {   // Start of the MTData2 message
        char length = data[1];
        parseMTData2(data + 2, length);
    } else {
        char length = data[2];
        uint16_t temp = ((uint16_t)data[1] | ((uint16_t)data[0] << 8)) & (uint16_t)0xFFFF;
        switch (temp) {                     // Extract the 2-byte Xsens Data Identifier
            case (uint16_t)DataID::TEMPERATURE:
                dataswapendian(data + 3, sizeof(float));
                memcpy(&temperature, data + 3, sizeof(float));
                break;
            case (uint16_t)DataID::PACKET_COUNTER:
                packet_count = (uint8_t)data[3] << 8 | (uint8_t)data[4];
                break;
            case (uint16_t)DataID::SAMPLE_TIME_FINE:
                sample_time_fine = (uint8_t)data[3] << 24 | (uint8_t)data[4] << 16 | (uint8_t)data[5] << 8 | (uint8_t)data[6];
                break;
            case (uint16_t)DataID::SAMPLE_TIME_COARSE:
                sample_time_coarse = (uint8_t)data[3] << 24 | (uint8_t)data[4] << 16 | (uint8_t)data[5] << 8 | (uint8_t)data[6];
                break;
            case (uint16_t)DataID::QUATERNION:
                dataswapendian(data + 3, sizeof(float) * 4);
                memcpy(quat, data + 3, sizeof(float) * 4);
                break;
            case (uint16_t)DataID::ROTATION_MATRIX:
                dataswapendian(data + 3, sizeof(float) * 9);
                memcpy(rot_matrix, data + 3, sizeof(float) * 9);
                break;
            case (uint16_t)DataID::EULER_ANGLES:
                dataswapendian(data + 3, sizeof(float) * 3);
                memcpy(euler, data + 3, sizeof(float) * 3);
                break;
            case (uint16_t)DataID::DELTA_V:
                dataswapendian(data + 3, sizeof(float) * 3);
                memcpy(deltav, data + 3, sizeof(float) * 3);
                break;
            case (uint16_t)DataID::ACCELERATION:
                dataswapendian(data + 3, sizeof(float) * 3);
                memcpy(acc, data + 3, sizeof(float) * 3);
                break;
            case (uint16_t)DataID::FREE_ACCELERATION:
                dataswapendian(data + 3, sizeof(float) * 3);
                memcpy(accfree, data + 3, sizeof(float) * 3);
                break;
            case (uint16_t)DataID::ACCELERATION_HR:
                dataswapendian(data + 3, sizeof(float) * 3);
                memcpy(acchr, data + 3, sizeof(float) * 3);
                break;
            case (uint16_t)DataID::RATE_OF_TURN:
                dataswapendian(data + 3, sizeof(float) * 3);
                memcpy(rot, data + 3, sizeof(float) * 3);
                break;
            case (uint16_t)DataID::DELTA_Q:
                dataswapendian(data + 3, sizeof(float) * 4);
                memcpy(deltaq, data + 3, sizeof(float) * 4);
                break;
            case (uint16_t)DataID::RATE_OF_TURN_HR:
                dataswapendian(data + 3, sizeof(float) * 3);
                memcpy(rothr, data + 3, sizeof(float) * 3);
                break;
            case (uint16_t)DataID::MAGNETIC_FIELD:
                dataswapendian(data + 3, sizeof(float) * 3);
                memcpy(mag, data + 3, sizeof(float) * 3);
                break;
            case (uint16_t)DataID::STATUS_BYTE:
                status_byte = (uint8_t)data[3];
                break;
            case (uint16_t)DataID::STATUS_WORD:
                status_word = (uint8_t)data[3] << 24 | (uint8_t)data[4] << 16 | (uint8_t)data[5] << 8 | (uint8_t)data[6];
                break;
            default:
                break;
        }
        parseMTData2(data + length + 3, datalength - length - 3); // Move onto next data element within MTData2 packet
    }
}

// Swap the endianness of the data such that the float value can be printed
void Xbus::dataswapendian(char *data, const uint8_t length) { 
    // creating arrays w/o set dimensions isn't recommended under C11 standards
    uint8_t cpy[length];       // Create a copy of the data
    memcpy(cpy, data, length); // Create a copy of the data
    for (int i = 0; i < length / 4; i++) {
        for (int j = 0; j < 4; j++) {
            data[j + i * 4] = cpy[3 - j + i * 4]; // Within each 4-byte (32-bit) float, reverse the order of the individual bytes
        }
    }
}

void Xbus::sendMessage(char *message, uint8_t numBytes) {
    // Compute the checksum for the Xbus message to be sent. See https://mtidocs.xsens.com/messages for details.
    uint8_t checksum = 0x01;
    for (int i = 1; i < numBytes; i++) {
        message[i] = (char)((uint8_t)message[i]);
        checksum -= message[i];
    }
    message[numBytes - 1] = checksum; // Add the checksum at the end of the Xbus message

    i2c_write_device(gpio_commander, i2c_handle, message, numBytes);

    if (print_raw) {
        std::cout << "Sent Msg (Hex): ";
        for (int i = 0; i < numBytes; i++) {
            std::cout << "0x" << std::setfill('0') << std::setw(2) << std::right << std::hex << (int)((uint8_t)message[i]);
            if (i < numBytes-1) {
                std::cout << ":";
            }
        }
        std::cout << std::endl;
    }
}