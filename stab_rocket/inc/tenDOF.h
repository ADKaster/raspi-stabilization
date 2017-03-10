
#ifndef TENDOF_H
#define TENDOF_H

#define TRUE (1)
#define FALSE (0)

#include <string>
#include <stdint.h>
#include "rocket/vector_raw_t.hpp"
#include "lsm303.h"
#include "l3gd20.h"

#define L3GD20_ADDR     (0x6B)
#define LSM303_MAG_ADDR (0x3C >> 1) 
#define LSM303_ACC_ADDR (0x32 >> 1)
#define BMP180_ADDR     (0x00)

typedef enum {
    L3GD20_IDX     = 0,
    LSM303_MAG_IDX = 1,
    LSM303_ACC_IDX = 2,
    BMP180_IDX     = 3,
    TENDOF_NUMDEV  = 4,
} tendof_idx_t;


class tenDOF_driver
{
private:
    static uint8_t devAddrs[TENDOF_NUMDEV];
    char useSmbus[TENDOF_NUMDEV];
    int  fd;

    int i2cWrite(uint8_t devIdx, uint8_t *data, int size);
    int i2cRead(uint8_t devIdx, uint8_t reg_start, uint8_t *data, int size);

    void init_accel(void);
    void init_mag(void);
    void init_gyro(void);
    void init_press(void);

    int read_accel(rocket::vector_raw_t *raw);

public:
    tenDOF_driver(char *filename);

    int run(void);
};


#endif /* TENDOF_H */
