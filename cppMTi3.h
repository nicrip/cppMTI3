#ifndef MTi3_h
#define MTi3_h

#include "Xbus.h"

class MTi3 {
    public:
        MTi3();
        ~MTi3();
        void printDebugMessages(bool set);
        bool detect(unsigned int timeout_ms);
        void goToConfig();
        void goToMeasurement();
        void printDeviceInfo();
        void runPrintSelfTest();
        void setDefaultOptionFlags();
        void printOptionFlags();
        void setLocationID(uint8_t id_hi, uint8_t id_lo);
        void printLocationID();
        void printAvailableFilterProfiles();
        void printCurrentFilterProfile();
        void setFilterProfile(unsigned int profile_id);
        void restoreFactoryDefaults();
        void reset();
        void setDefaultMeasurementOutputConfiguration();
        void setDefaultMagneticOutputConfiguration();
        void printOutputConfiguration();
        void printConfiguration();
        bool readMeasurement();
        bool readLogMeasurement(std::fstream* fs);
        void logAck(std::fstream* fs);
        bool waitUntilMessage(uint8_t id, unsigned int timeout_ms);
        void printeMTS();
        float getTemperature() {
            return xbus->temperature;
        }
        uint16_t getPacketCount() {
            return xbus->packet_count;
        }
        uint32_t getSampleTimeFine() {
            return xbus->sample_time_fine;
        }
        uint32_t getSampleTimeCoarse() {
            return xbus->sample_time_coarse;
        }
        float* getQuaternion() {
            return xbus->quat;
        }
        float* getRotationMatrix() {
            return xbus->rot_matrix;
        }
        float* getEulerAngles() {
            return xbus->euler;
        }
        float* getDeltaV() {
            return xbus->deltav;
        }
        float* getAcceleration() {
            return xbus->acc;
        }
        float* getFreeAcceleration() {
            return xbus->accfree;
        }
        float* getHighRateAcceleration() {
            return xbus->acchr;
        }
        float* getRateOfTurn() {
            return xbus->rot;
        }
        float* getDeltaQ() {
            return xbus->deltaq;
        }
        float* getHighRateRateOfTurn() {
            return xbus->rothr;
        }
        float* getMagneticField() {
            return xbus->mag;
        }
        uint8_t getStatusByte() {
            return xbus->status_byte;
        }
        uint32_t getStatusWord() {
            return xbus->status_word;
        }


    private:
        uint8_t address;
        Xbus* xbus;
        int gpio_commander;
        int i2c_handle;

        int getHandle() {
            return xbus->i2c_handle;
        }
        void sendMessage(char *message, uint8_t numBytes);
};

#endif
