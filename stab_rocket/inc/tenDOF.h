
#ifndef TENDOF_H
#define TENDOF_H

#define TRUE (1)
#define FALSE (0)

#include <string>
#include <stdint.h>

#define L3GD20_ADDR     (0x00)
#define LSM303_CMP_ADDR (0x00)
#define LSM303_ACC_ADDR (0x00)
#define BMP180_ADDR     (0x00)

typedef enum {
    L4GD20_IDX     = 0,
    LSM303_CMP_IDX = 1,
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

    int i2cWrite(uint8_t devIdx, uint8_t regAddr, uint8_t *data, int size);
    int i2cRead(uint8_t devIdx, uint8_t regAddr, uint8_t *data, int size);

public:
    tenDOF_driver(char *filename);

    int run(void);
};

#endif /* TENDOF_H */
