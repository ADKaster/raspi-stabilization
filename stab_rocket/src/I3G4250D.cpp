#include "I3G4250D.h"
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
#include <time.h>
}
#include <lcm/lcm-cpp.hpp>
#include "rocket/vector_t.hpp"
#include "rocket/vector_raw_t.hpp"

#define SENSITIVITY_245DPS (8.75) /* See page 10 of datasheet, typical is 245 */
#define DPS_TO_RAD         (0.017453293) /* Degrees/sec to rad/sec */


I3G4250D_driver::I3G4250D_driver(char *filename)
{
    uint64_t funcs;

    if ((this->fd = open(filename, O_RDWR | O_NONBLOCK)) < 0)
    {
        std::cout << "Failed to open the I2C bus" << std::endl;
        std::cout << "Did you use sudo? Proper device is /dev/i2c-1" << std::endl;
        exit(1);
    }

    if (ioctl(this->fd, I2C_FUNCS, &funcs) < 0)
    {
        std::cout << "Failed to get I2C_FUNC ioctl" << std::endl;
        exit(1);
    }

    if (funcs & I2C_FUNC_I2C)
    {
        this->useSmbus = FALSE;
        std::cout << "I2C Bus supports I2C_RDWR" << std::endl;
    }
    else if (funcs & I2C_FUNC_SMBUS_WORD_DATA)
    {
        this->useSmbus = TRUE;
        std::cout << "I2C Bus only supports SMBUS" << std::endl;
    }
    else
    {
        std::cout << "Invalid I2C funcs: " << std::hex << funcs << std::dec << std::endl;
    }
}

I3G4250D_driver::~I3G4250D_driver()
{
	close(fd);
}

/* Returns either -errno if there was an issue, OR the number of messages processed */
int I3G4250D_driver::i2cWrite(uint8_t devAddr, uint8_t *data, int size)
{
    int retVal = 0;
    if (useSmbus)
    {
        std::cout << "Remind Andrew to implement SMBUS" << std::endl;
        retVal = -1;
    }
    else
    {
        struct i2c_msg messages[1];

        messages[0].addr = devAddr;
        messages[0].buf = data;
        messages[0].len = size;
        messages[0].flags = 0;

        struct i2c_rdwr_ioctl_data payload;
        payload.msgs = messages;
        payload.nmsgs = sizeof(messages) / sizeof(messages[0]);
        
        retVal = ioctl(fd, I2C_RDWR, &payload);
        if (retVal < 0)
        {
            retVal = -errno;
#ifdef DEBUG_V
            std::cout << "Could not use I2C_RDWR on device " << devAddr<< std::endl;
            std::cout << "\terrno: " << errno << std::endl; 
#endif
        }
    }
    return retVal;
}

/* Returns either -errno if there was an issue, OR the number of messages processed */
int I3G4250D_driver::i2cRead(uint8_t devAddr, uint8_t regStart, uint8_t *data, int size)
{
    int retVal = 0;

    if(useSmbus)
    {
        std::cout << "Remind Andrew to implement I2C read with SMBUS" << std::endl;
        retVal = -1;
    }
    else
    {
        struct i2c_msg messages[2];

        messages[0].addr = devAddr;
        messages[0].flags = 0;
        messages[0].len = 1;
        messages[0].buf = &regStart;

        messages[1].addr = devAddr;
        messages[1].flags = I2C_M_RD;
        messages[1].buf = data;
        messages[1].len = size;

        struct i2c_rdwr_ioctl_data payload;
        payload.msgs = messages;
        payload.nmsgs = sizeof(messages) / sizeof(messages[0]);
        
        retVal = ioctl(fd, I2C_RDWR, &payload);
        if(retVal < 0)
        {
            retVal = -errno;
#ifdef DEBUG_V
            std::cout << "Could not use I2C_RDWR on device " << devAddr << std::endl;
            std::cout << "\terrno: " << errno << std::endl; 
#endif
        }
    }
    return retVal;
}


void I3G4250D_driver::init_gyro(void)
{
    /* Enable all 3 axes at 100Hz, 25 cutoff (?), see datasheet pg 30 */
    uint8_t msg[] = {I3G4250D_CTRL_REG1 , 0x3F};
    i2cWrite(I3G4250D_ADDR, msg, 2);


    /* Normal High pass filter, cutoff freq 8Hz  */
    msg[0] = I3G4250D_CTRL_REG2;
    msg[1] = 0x00;
    i2cWrite(I3G4250D_ADDR, msg, 2);

    /* No interrupts enabled  */
    msg[0] = I3G4250D_CTRL_REG3;
    msg[1] = 0x00;
    i2cWrite(I3G4250D_ADDR, msg, 2);

    /* LSB first, 245 DPS, No self test, Don't care for SPI  */
    msg[0] = I3G4250D_CTRL_REG4;
    msg[1] = 0x00;
    i2cWrite(I3G4250D_ADDR, msg, 2);

    /* see page 34 of datasheet */
    /* Normal Boot mode, No FIFO, no High pass filter (OVERRIDES CTRL REG 2), No Filtering at all  */
    msg[0] = I3G4250D_CTRL_REG5;
    msg[1] = 0x00;
    i2cWrite(I3G4250D_ADDR, msg, 2);

    std::cout << "I3G4250D Gyroscope initalized" << std::endl;   
}

int I3G4250D_driver::read_gyro(rocket::vector_t *gyro, rocket::vector_raw_t *raw)
{
    int retVal = 0;
    uint8_t buffer[6] = {0};
    uint8_t reg_start = (I3G4250D_OUT_X_L | 0x80); //want to do multi-read starting at this register

    retVal = i2cRead(I3G4250D_ADDR, reg_start, buffer, 6);
#ifdef DEBUG
    std::cout << "i2cRead retVal: " << retVal << std::endl;
#endif
    if(retVal >= 0)
    {

       // rocket::vector_raw_t raw;

        raw->x = (int16_t)(buffer[0] | (int16_t)(buffer[1] << 8));
        raw->y = (int16_t)(buffer[2] | (int16_t)(buffer[3] << 8));
        raw->z = (int16_t)(buffer[4] | (int16_t)(buffer[5] << 8));

        gyro->x = (double)raw->x * SENSITIVITY_245DPS * DPS_TO_RAD;
        gyro->y = (double)raw->y * SENSITIVITY_245DPS * DPS_TO_RAD;
        gyro->z = (double)raw->z * SENSITIVITY_245DPS * DPS_TO_RAD;
    }

    return retVal;
}

int I3G4250D_driver::run(void)
{
    lcm::LCM lcm;
    if(!lcm.good())
    {
        return 1;       
    }
    
    struct timespec prev_time;
    struct timespec curr_time;

    int64_t prev_ms;
    int64_t curr_ms;

    if(clock_gettime(CLOCK_MONOTONIC, &prev_time)){
        std::cout << "Could not get prev_time" << std::endl;
    }
    if(clock_gettime(CLOCK_MONOTONIC, &curr_time)){
        std::cout << "Could not get curr_time" << std::endl;
    }
    prev_ms = prev_time.tv_sec * 1000 + prev_time.tv_nsec / 1000000;
    curr_ms = curr_time.tv_sec * 1000 + curr_time.tv_nsec / 1000000;

    /* write initialization values */
    init_gyro();

    for(;;)
    {
        if(clock_gettime(CLOCK_MONOTONIC, &curr_time)){
            std::cout << "Could not get curr_time" << std::endl;
            continue;
        }        
        curr_ms = curr_time.tv_sec * 1000 + curr_time.tv_nsec / 1000000;

        /* do at 100Hz */
        if((curr_ms - prev_ms) >= 10)
        {
#ifdef DEBUG_V
            std::cout << (curr_ms - prev_ms) << std::endl;
#endif
            prev_ms = curr_ms;
            pubData.time = curr_ms;

            if(read_gyro(&(pubData.rad_s), &(pubData.raw)) < 0){
#ifdef DEBUG
                std::cout << "Invalid gyroscope data" << std::endl;
#endif
                pubData.rad_s.x = 0.0;
                pubData.rad_s.y = 0.0;
                pubData.rad_s.z = 0.0;                
            }

            lcm.publish("GYRO", &pubData);
        }/* End of 10ms check*/
    }/* End of infinite loop */
    return 0;
}
