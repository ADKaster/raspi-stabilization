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
#include <wiringPi.h>

/* Calibration notes: http://electroiq.com/blog/2010/11/introduction-to-mems-gyroscopes/ */

#define SENSITIVITY_245DPS (0.00875) /* See page 10 of datasheet, typical is 8.75 mDPS/digit */
#define DPS_TO_RAD         (0.017453293) /* Degrees/sec to rad/sec */


/****** GLOBALS *******/
volatile uint32_t isDataReady = FALSE;

/* ISR Routine on GPIO 11 (pin 23!!!), but WiringPi pin 14 -.- */
#define GYRO_GPIO_DATARDY (14)
void dataReadyISR(void)
{
    isDataReady = TRUE;
#ifdef DEBUG
    std::cout << "Hi from Interrupt!" << std::endl;
#endif
}

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

    /* Data Ready interrupt enabled only  */
    msg[0] = I3G4250D_CTRL_REG3;
    msg[1] = 0x0F;
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
    static int32_t zero_rate_data[3][100];
    static int32_t x_zr = 0, y_zr = 0, z_zr = 0;
    static int16_t num_samples_collected = 0;

    int retVal = 0;

    uint8_t status_reg = I3G4250D_STATUS_REG;
    uint8_t status_reg_val = 0;

    retVal = i2cRead(I3G4250D_ADDR, status_reg, &status_reg_val, 1);
    if(!((status_reg_val & 0x0F) == 0x0F))
    {
        std::cout << retVal << " Data value not ready!" << "status reg value: " << status_reg_val + 1 << std::endl;
        usleep(1000);
        status_reg = I3G4250D_STATUS_REG;
        retVal = i2cRead(I3G4250D_ADDR, status_reg, &status_reg_val, 1);
        std::cout << retVal << " Status reg try 2: " << status_reg_val + 1 << std::endl;
        usleep(1000);
        return 1000;
    }

    uint8_t buffer[6] = {0};
    uint8_t reg_start = (I3G4250D_OUT_X_L | 0x80); //want to do multi-read starting at this register

    retVal = i2cRead(I3G4250D_ADDR, reg_start, buffer, 6);
#ifdef DEBUG
    std::cout << "i2cRead retVal: " << retVal << std::endl;
#endif
    if(retVal >= 0)
    {

        raw->x = (int16_t)(buffer[0] | (int16_t)(buffer[1] << 8));
        raw->y = (int16_t)(buffer[2] | (int16_t)(buffer[3] << 8));
        raw->z = (int16_t)(buffer[4] | (int16_t)(buffer[5] << 8));

        gyro->x = (double)(raw->x - x_zr) * SENSITIVITY_245DPS * DPS_TO_RAD;
        gyro->y = (double)(raw->y - y_zr) * SENSITIVITY_245DPS * DPS_TO_RAD;
        gyro->z = (double)(raw->z - z_zr) * SENSITIVITY_245DPS * DPS_TO_RAD;

        /* Collect initial calibration data */
        if(num_samples_collected < 100)
        {
            zero_rate_data[0][num_samples_collected] = raw->x;
            zero_rate_data[1][num_samples_collected] = raw->y;
            zero_rate_data[2][num_samples_collected] = raw->z;
            num_samples_collected++;
        }
        else if(num_samples_collected == 100)
        {
            /* Average calibration data for zero rate level */
            for(int i = 0; i < 100; i++)
            {
                x_zr += zero_rate_data[0][i];
                y_zr += zero_rate_data[1][i];
                z_zr += zero_rate_data[2][i];
            }
            std::cout << "X, Y, Z zero rate sums: " << x_zr << " " << y_zr << " " << z_zr << std::endl;
            x_zr = (double)x_zr / 100;
            y_zr = (double)y_zr / 100;
            z_zr = (double)z_zr / 100;
            std::cout << "X, Y, Z zero rates: " << x_zr << " " << y_zr << " " << z_zr << std::endl;
            num_samples_collected++;
        }
    }

    return retVal;
}

int I3G4250D_driver::run(void)
{

    struct timespec curr_time;
    int64_t curr_ms;

    int is_error;
    int num_errors = 0, num_success = 0;

    lcm::LCM lcm;
    if(!lcm.good())
    {
        return 1;       
    }
    
    wiringPiSetup();

    wiringPiISR(GYRO_GPIO_DATARDY, INT_EDGE_BOTH, &dataReadyISR);

    /* write initialization values */
    init_gyro();

    for(;;)
    {
        if(clock_gettime(CLOCK_MONOTONIC, &curr_time)){
            std::cout << "Could not get curr_time" << std::endl;
            continue;
        }        
        curr_ms = (curr_time.tv_sec * 1000) + (curr_time.tv_nsec / 1000000);

        /* do when interrupt has triggered */
        if(isDataReady == TRUE)
        {
            isDataReady = FALSE;
            pubData.time = curr_ms;

            is_error = read_gyro(&(pubData.rad_s), &(pubData.raw));

            if(is_error < 0){
                pubData.rad_s.x = 0.0;
                pubData.rad_s.y = 0.0;
                pubData.rad_s.z = 0.0;
#ifdef DEBUG
                num_errors++;
                std::cout << "Invalid gyroscope data" << std::endl;
                std::cout << curr_ms << "| error  " << -is_error << "| error count "<< num_errors << "| success count " << num_success << std::endl;                
            }
            else
            {
                num_success++;
#endif
            }
            if(is_error == 2)
            {
                lcm.publish("GYRO", &pubData);
            }
        }/* End of check interrupt*/
    }/* End of infinite loop */
    return 0;
}
