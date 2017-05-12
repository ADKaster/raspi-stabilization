#ifndef I3G4250D_H
#define I3G4250D_H

#include <string>
#include <stdint.h>
#include "rocket/vector_t.hpp"
#include "rocket/vector_raw_t.hpp"
#include "rocket/gyro_t.hpp"

/* Table 18 of http://www.st.com/content/ccc/resource/technical/document/datasheet/e4/b1/d1/62/1a/e6/44/2f/DM00168691.pdf/files/DM00168691.pdf/jcr:content/translations/en.DM00168691.pdf */

/*  Register Name         | R/W |   Address 
 *	Reserved				-		0x00-0E
 *	WHO_AM_I				r		0x0F
 *	Reserved				-		0x10-1F
 *	CTRL_REG1				rw		0x20
 *	CTRL_REG2				rw		0x21
 *	CTRL_REG3				rw		0x22
 *	CTRL_REG4				rw		0x23
 *	CTRL_REG5				rw		0x24
 *	REFERENCE/DATACAPTURE	rw		0x25
 *	OUT_TEMP				r		0x26
 *	STATUS_REG				r		0x27
 *	OUT_X_L					r		0x28
 *	OUT_X_H					r		0x29
 *	OUT_Y_L					r		0x2A
 *	OUT_Y_H					r		0x2B
 *	OUT_Z_L					r		0x2C
 *	OUT_Z_H					r		0x2D
 *	FIFO_CTRL_REG			rw		0x2E
 *	FIFO_SRC_REG			r		0x2F
 *	INT1_CFG				rw		0x30
 *	INT1_SRC				r		0x31
 *	INT1_THS_XH				rw		0x32
 *	INT1_THS_XL				rw		0x33
 *	INT1_THS_YH				rw		0x34
 *	INT1_THS_YL				rw		0x35
 *	INT1_THS_ZH				rw		0x36
 *	INT1_THS_ZL				rw		0x37
 *	INT1_DURATION			rw		0x38
 */

#define I3G4250D_WHO_AM_I		(0x0F)
#define I3G4250D_CTRL_REG1  	(0x20)
#define I3G4250D_CTRL_REG2		(0x21)
#define I3G4250D_CTRL_REG3		(0x22)
#define I3G4250D_CTRL_REG4		(0x23)
#define I3G4250D_CTRL_REG5		(0x24)
#define I3G4250D_REF_DATA		(0x25)
#define I3G4250D_OUT_TEMP		(0x26)
#define I3G4250D_STATUS_REG		(0x27)
#define I3G4250D_OUT_X_L		(0x28)
#define I3G4250D_OUT_X_H		(0x29)
#define I3G4250D_OUT_Y_L		(0x2A)
#define I3G4250D_OUT_Y_H		(0x2B)
#define I3G4250D_OUT_Z_L		(0x2C)
#define I3G4250D_OUT_Z_H		(0x2D)
#define I3G4250D_FIFO_CTRL_REG	(0x2E)
#define I3G4250D_FIFO_SRC_REG	(0x2F)
#define I3G4250D_INT1_CFG		(0x30)
#define I3G4250D_INT1_SRC		(0x31)
#define I3G4250D_INT1_THS_XH	(0x32)
#define I3G4250D_INT1_THS_XL	(0x33)
#define I3G4250D_INT1_THS_YH	(0x34)
#define I3G4250D_INT1_THS_YL	(0x35)
#define I3G4250D_INT1_THS_ZH	(0x36)
#define I3G4250D_INT1_THS_ZL	(0x37)
#define I3G4250D_INT1_DURATION	(0x38)

#define I3G4250D_ADDR (0x69)
#define TRUE (1)
#define FALSE (0)

class I3G4250D_driver
{
private:
    bool useSmbus;
    int  fd;
    rocket::gyro_t pubData;

    int i2cWrite(uint8_t devAddr, uint8_t *data, int size);
    int i2cRead(uint8_t devAddr, uint8_t reg_start, uint8_t *data, int size);

    void init_gyro(void);

    int read_gyro(rocket::vector_t *gyro, rocket::vector_raw_t *raw);

public:
    I3G4250D_driver(char *filename);
    ~I3G4250D_driver();

    int run(void);
};

#endif /* I3G4250D_H */