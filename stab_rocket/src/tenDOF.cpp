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
#include "rocket/vector_raw_t.hpp"

uint8_t tenDOF_driver::devAddrs[TENDOF_NUMDEV] = 
        {L3GD20_ADDR, LSM303_MAG_ADDR, LSM303_ACC_ADDR, BMP180_ADDR};

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

int tenDOF_driver::i2cWrite(uint8_t devIdx, uint8_t *data, int size)
{
    int retVal = 0;
    if (useSmbus[devIdx])
    {
        std::cout << "Remind Andrew to implement SMBUS" << std::endl;
        retVal = -1;
    }
    else
    {
        struct i2c_msg messages[1] = {0};
        messages[0].addr = devAddrs[devIdx];
        messages[0].buf = data;
        messages[0].len = size;

        struct i2c_rdwr_ioctl_data payload;
        payload.msgs = messages;
        payload.nmsgs = sizeof(messages) / sizeof(messages[0]);
        
        retVal = ioctl(fd, I2C_RDWR, &payload);
        if (retVal < 0)
        {
            retVal = -errno;
            std::cout << "Could not use I2C_RDWR on device " << devIdx << std::endl;
        }
    }
    return retVal;
}

int tenDOF_driver::i2cRead(uint8_t devIdx, uint8_t regStart, uint8_t *data, int size)
{
    int retVal = 0;

    if(useSmbus[devIdx])
    {
        std::cout << "Remind Andrew to implement I2C read with SMBUS" << std::endl;
        retVal = -1;
    }
    else
    {
        struct i2c_msg messages[2];

        messages[0].addr = devAddrs[devIdx];
        messages[0].len = 1;
        messages[0].buf = &regStart;

        messages[1].addr = devAddrs[devIdx];
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
            std::cout << "Could not use I2C_RDWR on device " << devIdx << std::endl;
        }
    }
    return retVal;
}

void tenDOF_driver::init_accel(void)
{
    uint8_t msg[] = {LSM303_REGISTER_ACCEL_CTRL_REG1_A , 0x57};
    i2cWrite(LSM303_ACC_IDX, msg, 2);
    
    std::cout << "accelerometer initalized" << std::endl;   
}

void tenDOF_driver::init_mag(void)
{
    uint8_t msg[] = {LSM303_REGISTER_MAG_CRA_REG_M, 0x00};
    i2cWrite(LSM303_MAG_IDX, msg, 2);

    msg[0] = LSM303_REGISTER_MAG_CRB_REG_M;
    msg[1] = LSM303_MAGGAIN_1_3;
    i2cWrite(LSM303_MAG_IDX, msg, 2);

    std::cout << "magnetometer initalized" << std::endl;
}

void tenDOF_driver::init_gyro(void)
{
    uint8_t msg[] = {GYRO_REGISTER_CTRL_REG1, 0x00};
    /* Set CTRL_REG1 (0x20)
    ====================================================================
    BIT  Symbol    Description                                   Default
    ---  ------    --------------------------------------------- -------
    7-6  DR1/0     Output data rate                                   00
    5-4  BW1/0     Bandwidth selection                                00
     3  PD        0 = Power-down mode, 1 = normal/sleep mode          0
     2  ZEN       Z-axis enable (0 = disabled, 1 = enabled)           1
     1  YEN       Y-axis enable (0 = disabled, 1 = enabled)           1
     0  XEN       X-axis enable (0 = disabled, 1 = enabled)           1 */

    /* Reset then switch to normal mode and enable all three channels */
    i2cWrite(L3GD20_IDX, msg, 2);

    msg[0] = GYRO_REGISTER_CTRL_REG1;
    msg[1] = 0x0F;
    i2cWrite(L3GD20_IDX, msg, 1);
    /* ------------------------------------------------------------------ */
    
    /* Set CTRL_REG2 (0x21)
    ====================================================================
    BIT  Symbol    Description                                   Default
    ---  ------    --------------------------------------------- -------
    5-4  HPM1/0    High-pass filter mode selection                    00
       3-0  HPCF3..0  High-pass filter cutoff frequency selection      0000 */
    /* Nothing to do ... keep default values */
    /* ------------------------------------------------------------------ */

    /* Set CTRL_REG3 (0x22)
       ====================================================================
       BIT  Symbol    Description                                   Default
       ---  ------    --------------------------------------------- -------
         7  I1_Int1   Interrupt enable on INT1 (0=disable,1=enable)       0
         6  I1_Boot   Boot status on INT1 (0=disable,1=enable)            0
         5  H-Lactive Interrupt active config on INT1 (0=high,1=low)      0
         4  PP_OD     Push-Pull/Open-Drain (0=PP, 1=OD)                   0
         3  I2_DRDY   Data ready on DRDY/INT2 (0=disable,1=enable)        0
         2  I2_WTM    FIFO wtrmrk int on DRDY/INT2 (0=dsbl,1=enbl)        0
         1  I2_ORun   FIFO overrun int on DRDY/INT2 (0=dsbl,1=enbl)       0
         0  I2_Empty  FIFI empty int on DRDY/INT2 (0=dsbl,1=enbl)         0 */
    
    /* Nothing to do ... keep default values */
    /* ------------------------------------------------------------------ */    

    /* Set CTRL_REG4 (0x23)
    ====================================================================
    BIT  Symbol    Description                                   Default
    ---  ------    --------------------------------------------- -------
        7  BDU       Block Data Update (0=continuous, 1=LSB/MSB)         0
        6  BLE       Big/Little-Endian (0=Data LSB, 1=Data MSB)          0
        5-4  FS1/0     Full scale selection                               00
                                    00 = 250 dps
                                    01 = 500 dps
                                    10 = 2000 dps
                                    11 = 2000 dps
        0  SIM       SPI Mode (0=4-wire, 1=3-wire)                       0 */
    msg[0] = GYRO_REGISTER_CTRL_REG4;
    msg[1] = 0x10;
    i2cWrite(L3GD20_IDX, msg, 1);
    /* ------------------------------------------------------------------ */    
    /* Set CTRL_REG5 (0x24)
    ====================================================================
    BIT  Symbol    Description                                   Default
    ---  ------    --------------------------------------------- -------
        7  BOOT      Reboot memory content (0=normal, 1=reboot)          0
        6  FIFO_EN   FIFO enable (0=FIFO disable, 1=enable)              0
        4  HPen      High-pass filter enable (0=disable,1=enable)        0
        3-2  INT1_SEL  INT1 Selection config                              00
        1-0  OUT_SEL   Out selection config                               00 */    

    /* Nothing to do ... keep default values */
    /* ------------------------------------------------------------------ */
    
    std::cout << "Initialized gyroscope" << std::endl;
   
}

void tenDOF_driver::init_press(void)
{
    std::cout << "Initialized pressure sensor. jk it's unimplemented." << std::endl;
}

int tenDOF_driver::read_accel(rocket::vector_raw_t *raw)
{
    int retVal = 0;
    uint8_t buffer[6] = {0};
    uint8_t reg_start = (LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80); //want to read starting at this register

    retVal = i2cRead(LSM303_ACC_IDX, reg_start, buffer, 6);

/*   // Shift values to create properly formed integer (low byte first)
  raw.x = (int16_t)(xlo | (xhi << 8)) >> 4;
  raw.y = (int16_t)(ylo | (yhi << 8)) >> 4;
  raw.z = (int16_t)(zlo | (zhi << 8)) >> 4;*/


    if(retVal >= 0)
    {
        raw->x = (int16_t)(buffer[0] | (buffer[1] << 8)) >> 4;
        raw->y = (int16_t)(buffer[2] | (buffer[3] << 8)) >> 4;
        raw->z = (int16_t)(buffer[4] | (buffer[5] << 8)) >> 4;
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
    init_accel();
    init_mag();
    init_gyro();
    init_press();

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
