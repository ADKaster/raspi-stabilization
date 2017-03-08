#include "tenDOF.h"
#include <cstdio>
#include <iostream>
#include <unistd.h>
extern "C"
{
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
}
#include <lcm/lcm-cpp.hpp>
#include "rocket/tenDOF_t.hpp"
#include "rocket/vector_t.hpp"

uint8_t tenDOF_driver::devAddrs[TENDOF_NUMDEV] = 
        {L3GD20_ADDR, LSM303_CMP_ADDR, LSM303_ACC_ADDR, BMP180_ADDR};

tenDOF_driver::tenDOF_driver(char *filename)
{
    if ((this->fd = open(filename, O_RDWR | O_NONBLOCK)) < 0)
    {
        std::cout << "Failed to open the bus" << std::endl;
        exit(1);
    }
    uint64_t funcs;
    for (int i = 0; i < TENDOF_NUMDEV; i++)
    {
        if (ioctl(this->fd, I2C_FUNCS, &funcs) < 0)
        {
            std::cout << "Failed to get I2C_FUNC ioctl" << std::endl;
            exit(1);
        }
        if (funcs & I2C_FUNC_I2C)
        {
            this->useSmbus[i] = FALSE;
        }
        else if (funcs & I2C_FUNC_SMBUS_WORD_DATA)
        {
            this->useSmbus[i] = TRUE;
        }
        else
        {
            std::cout << "Invalid I2C funcs: " << std::hex << funcs << std::dec << std::endl;
        }
    }
}

int tenDOF_driver::i2cWrite(uint8_t devIdx, uint8_t regAddr, uint8_t *data, int size)
{
    int retVal = 0;
    /*Do use I2C_SMBUS  */
    /*if(useSmbus[devIdx])
    {
        struct i2c_smbus_ioctl_data payload;
        payload.read_write = I2C_SMBUS_WRITE;
        payload.size = I2C_SMBUS_WORD_DATA;
        payload.command = regAddr;
        payload.data = data;

        retVal = ioctl (fd, I2C_SLAVE_FORCE, devAddrs[devIdx]);
        if (retVal < 0)
        {
            retVal = -errno;
            std::cout << "Could not use I2C_SLAVE_FORCE on device" << devIdx << std::endl;
        }
        else
        {
            retVal = ioctl (fd, I2C_SMBUS, &payload);
            if (retVal < 0)
            {
                ret = -errno;
                std::cout << "Could not use I2C_SMBUS on device" << devIdx << std::endl;
            }
        }
    }
    // Use I2C_RDWR instead
    else
    {*/
        struct i2c_msg messages[1];
        messages[0].addr = devAddrs[devIdx];
        messages[0].buf = data;
        messages[0].len = size;

        struct i2c_rdwr_ioctl_data payload;
        payload.msgs = messages;
        payload.nmsgs = 1;
        
        retVal = ioctl(fd, I2C_RDWR, &payload);
        if (retVal < 0)
        {
            retVal = -errno;
            std::cout << "Could not use I2C_RDWR on device " << devIdx << std::endl;
        }
 //   }
    return retVal;
}

int tenDOF_driver::i2cRead(uint8_t devIdx, uint8_t regAddr, uint8_t *recvData, int size)
{
    int retVal = 0;

    if(useSmbus[devIdx])
    {
        std::cout << "Remind Andrew to implement I2C read with SMBUS" << std::endl;
        retVal = -1;
    }
    else
    {
        struct i2c_msg messages[1];
        messages[0].addr = devAddrs[devIdx];
        messages[0].flags = I2C_M_RD;
        messages[0].buf = recvData;
        messages[0].len = size;

        struct i2c_rdwr_ioctl_data payload;
        payload.msgs = messages;
        payload.nmsgs = sizeof(messages) / sizeof(messages[0]);
        
        retVal = ioctl(fd, I2C_RDWR, &payload);
        if(retVal < 0)
        {
            retVal = -errno;
            std::cout << "Could not use I2C_RDWR on device " << devIdx << std::endl;
        }
    }
    return retVal;
}

int tenDOF_driver::run(void)
{
    lcm::LCM lcm;
    if(!lcm.good())
    {
        return 1;       
    }
    
    rocket::tenDOF_t pubData;
    
    /* write initialization values */

    for(;;)
    {
        for(int i = 0; i < TENDOF_NUMDEV; i++)
        {
            /*read all data and convert into values */       
        }
        lcm.publish("TENDOF", &pubData);
        /*publish */
        /* wait for howerver many ms between reads */
    }
    return 0;
}
