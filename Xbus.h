#ifndef XBUS_H
#define XBUS_H

#include <pigpiod_if2.h>
#include <cstring>
#include <math.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <bitset>

// Definition of opcodes: For more information on opcodes, refer to https://mtidocs.xsens.com/functional-description$mtssp-synchronous-serial-protocol
#define XSENS_CONTROL_PIPE 0x03 // Use this opcode for sending (configuration) commands to the MTi
#define XSENS_STATUS_PIPE 0x04  // Use this opcode for reading the status of the notification/measurement pipes
#define XSENS_NOTIF_PIPE 0x05   // Use this opcode for reading a notification message
#define XSENS_MEAS_PIPE 0x06    // Use this opcode for reading measurement data (MTData2 message)

#define max_msg_size 5192

class Xbus {
    public:
        Xbus();
        Xbus(int pigpio_id, uint8_t I2C_ADDR, uint8_t I2C_BUS);
        // Closes I2C line
        ~Xbus();
        // Reads pipes and outputs message codes and returns true if measurement data was read
        bool read();
        uint8_t readUntilAck(uint8_t opcode, int timeout);
        void readPipeStatus();
        void readPipeNotif();
        void readPipeMeas();
        void parseMTData2(char *data, const uint8_t datalength);
        uint8_t parseNotification(char *notif, const uint16_t notiflength);
        // Swap the endianness of the data such that the float value can be printed
        void dataswapendian(char *data, uint8_t length);
        void parseSelfTestResults(std::bitset<16> bit_results);
        void parseError(int error_code);
        void parseOptionFlag(uint32_t option_flag);
        void parseDataIdentifier(uint16_t gtf);
        void sendMessage(char *message, uint8_t numBytes);
        
        // Acknowledgement Codes
        enum MesID {
            // Wake Up + State Messages
            WAKEUP = '\x3E',
            WAKEUPACK = '\x3F',
            GOTOCONFIG = '\x30',
            GOTOCONFIGACK = '\x31',
            GOTOMEASUREMENT = '\x10',
            GOTOMEASUREMENTACK = '\x11',
            RESET = '\x40',
            RESETACK = '\x41',
            
            // Informational Messages
            REQDID = '\x00',
            DEVICEID = '\x01',              // Response to REQID message
            REQPRODUCTCODE = '\x1C',
            PRODUCTCODE = '\x1D',           // Response to REQPRODUCTCODE message
            REQHARDWAREVERSION = '\x1E',
            HARDWAREVERSION = '\x1F',       // Response to REQHARDWAREVERSION message
            REQFWREV = '\x12',
            FIRMWAREREV = '\x13',           // Response to REQFWREV message
            RUNSELFTEST = '\x24',
            SELFTESTRESULTS = '\x25',       // Response to RUNSELFTEST message
            ERROR = '\x42',
            WARNING = '\x43',
            
            // Device-Specific Messages
            REQOPTIONFLAG = '\x48',
            SETOPTIONFLAG = '\x48',
            OPTIONFLAG = '\x49',            // Response to REQOPTIONFLAG message
            REQLOCATIONID = '\x84',
            SETLOCATIONID = '\x84',
            LOCATIONID = '\x85',            // Response to REQLOCATIONID message
            RESTOREFACTORYDEF = '\x0E',
            RESTOREFACTORYDEFACK = '\x0F',
            
            // Configuration Messages
            REQCONFIGURATION = '\x0C',
            CONFIGURATION = '\x0D',         // Response to REQCONFIGURATION message
            REQOUTPUTCONFIGURATION = '\xC0',
            SETOUTPUTCONFIGURATION = '\xC0',
            OUTPUTCONFIGURATION = '\xC1',   // Response to REQOUTPUTCONFIGURATION message
            
            // Data
            MTDATA2 = '\x36',
            ICCCOMANDACK = '\x75',

            // Filter Messages
            REQAVAILABLEFILTERPROFILES = '\x62',
            AVAILABLEFILTERPROFILES = '\x63',
            REQFILTERPROFILE = '\x64',
            SETFILTERPROFILE = '\x64',
            REQFILTERPROFILEACK = '\x65',

            //eMTS Messages
            REQEMTS = '\x90',
            EMTS = '\x91',
        };

        // Data codes, default system is Single precision IEEE 32-bit floating point number with
        // North-East-Down coordinates
        enum DataID {
            TEMPERATURE = 0x0810,
            UTC_TIME = 0x1010,
            PACKET_COUNTER = 0x1020,
            SAMPLE_TIME_FINE = 0x1060,
            SAMPLE_TIME_COARSE = 0x1070,
            QUATERNION = 0x2014,
            ROTATION_MATRIX = 0x2024,
            EULER_ANGLES = 0x2034,
            DELTA_V = 0x4010,
            ACCELERATION = 0x4024,
            FREE_ACCELERATION = 0x4034,
            ACCELERATION_HR = 0x4044,
            RATE_OF_TURN = 0x8020,
            DELTA_Q = 0x8030,
            RATE_OF_TURN_HR = 0x8040,
            MAGNETIC_FIELD = 0xC020,
            STATUS_BYTE = 0xE010,
            STATUS_WORD = 0xE020,
        };

        float temperature = NAN;            // Used to store Temperature reading
        uint16_t packet_count = 0;          // Used to store Packet Counter reading
        uint32_t sample_time_fine = 0;      // Used to store Sample Time Fine reading
        uint32_t sample_time_coarse = 0;    // Used to store Sample Time Coarse reading
        float quat[4] = {NAN};              // Used to store Quaternion reading
        float rot_matrix[9] = {NAN};        // Used to store rotation matrix
        float euler[3] = {NAN};             // Used to store latest EulerAngle reading
        float deltav[3] = {NAN};            // Used to store latest Delta V reading
        float acc[3] = {NAN};               // Used to store latest Acceleration reading
        float accfree[3] = {NAN};           // Used to store latest Free Acceleration reading
        float acchr[3] = {NAN};             // Used to store latest Acceleration HR reading
        float rot[3] = {NAN};               // Used to store latest RateOfTurn reading
        float deltaq[4] = {NAN};            // Used to store latest Delta Q reading
        float rothr[3] = {NAN};             // Used to store latest RateOfTurn HR reading
        float mag[3] = {NAN};               // Used to store latest Magnetic Field reading
        uint8_t status_byte = 0;            // Used to store latest Status Byte reading
        uint32_t status_word = 0;           // Used to store latest Status Word reading

        bool configState = false;           // True if MTi is in Config mode, false if MTi is in Measurement mode
        char productCode = 3;               // Contains the product code (MTi-#) of the connected device
        bool wakeState = false;

        int gpio_commander;                 // holds the pigpio_id
        int i2c_handle;                     // holds the i2c line handle for pigpio commands
        uint8_t I2C_ADDR_;
        uint8_t I2C_BUS_;

        bool print_raw;
        
        char datanotif[max_msg_size];       // Used to store content read from the Notification Pipe
        char datameas[max_msg_size];        // Used to store content read from the Measurement Pipe
        char datameas_copy[max_msg_size];   // Used to store content read from the Measurement Pipe
        char status[4];                     // Used to store indicators of the Status Pipe
        uint16_t notificationSize;
        uint16_t measurementSize;
};

#endif
