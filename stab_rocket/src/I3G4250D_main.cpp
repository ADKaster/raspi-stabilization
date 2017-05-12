#include "I3G4250D.h"

#include <iostream>

int main(int argc, char **argv)
{
    if(argc < 2)
    {
        std::cout << "usage: ./gyroscope /dev/i2c-1" << std::endl;
        return -1;
    }

    I3G4250D_driver gyro(argv[1]);

    return gyro.run();
}
    