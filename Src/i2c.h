/*
    i2c.h




 */

#ifndef _I2C_H_
#define _I2C_H_

void TWIInit(void);

uint8_t TWIStart(uint8_t address);

//send stop signal
void TWIStop(void);

uint8_t TWIWrite(uint8_t u8data);

uint8_t TWIReadACK(void);

//read byte with NACK
uint8_t TWIReadNACK(void);


uint8_t TWIGetStatus(void);


#define i2c_init TWIInit
#define i2c_start TWIStart
#define i2c_write TWIWrite
#define i2c_stop TWIStop

#endif //  _I2C_H_

