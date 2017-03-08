#include "tenDOF.h"

#include <iostream>

int main(int argc, char **argv)
{
    if(argc < 2)
    {
        std::cout << "usage: ./tenDOF /dev/i2c-1" << std::endl;
        return -1;
    }

    tenDOF_driver tendof(argv[1]);

    return tendof.run();
}
