#include "cppMTi3.h"
#include <fstream>

// Script to configure and run the MTi

int main(int argc, char *argv[])
{
    char test[10] = {'\xFF', '\xFA', '\x01', '\x02', '\x03', '\x04', '\x05', '\x06', '\x07', '\x08'};

    std::string filename;

    if (argc > 1) {
        filename = std::string(argv[1]) + ".mtb";
    } else {
        filename = "./maglog.mtb";
    }
    std::cout << filename.c_str() << std::endl;

    std::fstream fs(filename.c_str(), std::fstream::trunc | std::fstream::binary | std::fstream::out);
    fs.write(test, sizeof(test));
    fs.write(test, sizeof(test));
    fs.write(test, sizeof(test));
}