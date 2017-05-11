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
#include <time.h>
}
#include <lcm/lcm-cpp.hpp>
#include "rocket/tenDOF_t.hpp"
#include "rocket/vector_t.hpp"
#include "rocket/vector_raw_t.hpp"

/* Datasheet reference: */
/* https://cdn-shop.adafruit.com/datasheets/LSM303DLHC.PDF */

/* For documentation of I2C-dev ioctls, see https://www.kernel.org/doc/Documentation/i2c/dev-interface */


#define LSM303_ACC_SENS         (0.012F)    /* 12 mg/LSB per page 9 of datasheet */
#define G_TO_MS2                (9.80665F)  /* 1 g to m/s^2 */
#define LSM303_MAG_LSBGAUSS_XY  (670.0F)      /* LSB/Gauss. Depends on gain. see pg 9 of datasheet*/
#define LSM303_MAG_LSBGAUSS_Z   (600.0F)
#define DPS_TO_RAD              (0.017453293F) /* Degrees/sec to rad/sec */

uint8_t tenDOF_driver::devAddrs[TENDOF_NUMDEV] = 
        {L3GD20_ADDR, LSM303_MAG_ADDR, LSM303_ACC_ADDR, BMP180_ADDR};


tenDOF_driver::tenDOF_driver(char *filename)
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

tenDOF_driver::~tenDOF_driver()
{
    close(fd);
}

int tenDOF_driver::i2cWrite(uint8_t devIdx, uint8_t *data, int size)
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

        messages[0].addr = devAddrs[devIdx];
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
            std::cout << "Could not use I2C_RDWR on device " << devIdx << std::endl;
            std::cout << "\terrno: " << errno << std::endl; 
#endif
        }
    }
    return retVal;
}

int tenDOF_driver::i2cRead(uint8_t devIdx, uint8_t regStart, uint8_t *data, int size)
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

        messages[0].addr = devAddrs[devIdx];
        messages[0].flags = 0;
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
#ifdef DEBUG_V
            std::cout << "Could not use I2C_RDWR on device " << devIdx << std::endl;
            std::cout << "\terrno: " << errno << std::endl; 
#endif
        }
    }
    return retVal;
}

void tenDOF_driver::init_accel(void)
{
    /* 100Hz : enable all 3 axes see datasheet pg 24 */
    uint8_t msg[] = {LSM303_REGISTER_ACCEL_CTRL_REG1_A , 0x57};
    i2cWrite(LSM303_ACC_IDX, msg, 2);


    /* +- 16g output */
    msg[0] = LSM303_REGISTER_ACCEL_CTRL_REG4_A;
    msg[1] = 0x30;
    i2cWrite(LSM303_ACC_IDX, msg, 2);


    std::cout << "accelerometer initalized" << std::endl;   
}

void tenDOF_driver::init_mag(void)
{
    /* Set output data rate to 220 Hz*/
    uint8_t msg[] = {LSM303_REGISTER_MAG_CRA_REG_M, 0x1C};
    i2cWrite(LSM303_MAG_IDX, msg, 2);

    msg[0] = LSM303_REGISTER_MAG_CRB_REG_M;
    msg[1] = LSM303_MAGGAIN_2_5; /* +- 2.5 mG */
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

    /* Output data rate = 190Hz with 12.5 cutoff (cutoff what? idk) */
    msg[0] = GYRO_REGISTER_CTRL_REG1;
    msg[1] = 0x4F;
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

    /* 500DPS */
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

int tenDOF_driver::read_accel(rocket::vector_t *accel)
{
    int retVal = 0;
    uint8_t buffer[6] = {0};
    uint8_t reg_start = (LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80); //want to read starting at this register

    retVal = i2cRead(LSM303_ACC_IDX, reg_start, buffer, 6);

    if(retVal >= 0)
    {

        rocket::vector_raw_t raw;

        raw.x = (int16_t)(buffer[0] | (buffer[1] << 8)) >> 4;
        raw.y = (int16_t)(buffer[2] | (buffer[3] << 8)) >> 4;
        raw.z = (int16_t)(buffer[4] | (buffer[5] << 8)) >> 4;

        accel->x = (double)raw.x * LSM303_ACC_SENS * G_TO_MS2;
        accel->y = (double)raw.y * LSM303_ACC_SENS * G_TO_MS2;
        accel->z = (double)raw.z * LSM303_ACC_SENS * G_TO_MS2;
    }

    return retVal;
}

int tenDOF_driver::read_mag(rocket::vector_t *mag)
{
    int retVal = 0;
    uint8_t buffer[6] = {0};
    uint8_t reg_start = LSM303_REGISTER_MAG_OUT_X_H_M; /* Read starting at this register */

    retVal = i2cRead(LSM303_MAG_IDX, reg_start, buffer, 6);

    if(retVal >= 0)
    {
        rocket::vector_raw_t raw;

        /* X Z Y MSB then LSB */
        raw.x = (int16_t)(buffer[1] | (int16_t)(buffer[0] << 8));
        raw.z = (int16_t)(buffer[3] | (int16_t)(buffer[2] << 8));
        raw.y = (int16_t)(buffer[5] | (int16_t)(buffer[4] << 8));

        mag->x = (double)raw.x / LSM303_MAG_LSBGAUSS_XY;
        mag->y = (double)raw.y / LSM303_MAG_LSBGAUSS_XY;
        mag->z = (double)raw.z / LSM303_MAG_LSBGAUSS_Z;
    }

    return retVal;
}

int tenDOF_driver::read_gyro(rocket::vector_t *gyro)
{
    int retVal = 0;
    uint8_t buffer[6] = {0};
    uint8_t reg_start = (GYRO_REGISTER_OUT_X_L | 0x80); //want to read starting at this register

    retVal = i2cRead(LSM303_ACC_IDX, reg_start, buffer, 6);

    if(retVal >= 0)
    {

        rocket::vector_raw_t raw;

        raw.x = (int16_t)(buffer[0] | (int16_t)(buffer[1] << 8));
        raw.y = (int16_t)(buffer[2] | (int16_t)(buffer[3] << 8));
        raw.z = (int16_t)(buffer[4] | (int16_t)(buffer[5] << 8));

        gyro->x = (double)raw.x * GYRO_SENSITIVITY_500DPS * DPS_TO_RAD;
        gyro->y = (double)raw.y * GYRO_SENSITIVITY_500DPS * DPS_TO_RAD;
        gyro->z = (double)raw.z * GYRO_SENSITIVITY_500DPS * DPS_TO_RAD;
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
    init_accel();
    init_mag();
    init_gyro();
    init_press();

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
            if(read_accel(&(pubData.Accel))){
#ifdef DEBUG
                std::cout << "Invalid accelerometer data" << std::endl;
#endif
                pubData.Accel.x = 0.0;
                pubData.Accel.y = 0.0;
                pubData.Accel.z = 0.0;
            }
            if(read_mag(&(pubData.Mag))){
#ifdef DEBUG
                std::cout << "Invalid magnetometer data" << std::endl;
#endif
                pubData.Mag.x = 0.0;
                pubData.Mag.y = 0.0;
                pubData.Mag.z = 0.0;
            }
            if(read_gyro(&(pubData.Gyro))){
#ifdef DEBUG
                std::cout << "Invalid gyroscope data" << std::endl;
#endif
                pubData.Gyro.x = 0.0;
                pubData.Gyro.y = 0.0;
                pubData.Gyro.z = 0.0;                
            }

            lcm.publish("TENDOF", &pubData);
        }/* End of 10ms check*/
    }/* End of infinite loop */
    return 0;
}
