/*
    i2c.c

    Copyright 2020 GLecuyer <glecuyer207@gmail.com>


 */


#include <avr/io.h>
#include <util/twi.h>
#include <stdio.h>


#define F_I2C           100000UL  // clock i2c
#define PSC_I2C         1         // prescaler i2c
#define SET_TWBR        (F_CPU/F_I2C-16UL)/(PSC_I2C*2UL)

/*
 *
 * SCL_frequency = CPU_Clock_frequency / 16 + 2(TWBR) * PrescalerValue
 *
 * TWBR = CPU_Clock_frequency / SCL_frequency*2
 *
 *      = 16000000 / 2*100000
 *
***********************************************************************/
void TWIInit(void)
{
    //set SCL to 100kHz
    TWSR = 0x00;
    TWBR = (uint8_t)SET_TWBR;

}


/*
 *
 * I2C start and stop
 *
 **************************************************************/
uint8_t TWIStart(uint8_t address)
{
    // reset TWI control register
    TWCR = 0;
    // transmit START condition
    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
    // wait for end of transmission
    while( !(TWCR & (1<<TWINT)) )
        ;

    // check if the start condition was successfully transmitted
    if((TWSR & 0xF8) != TW_START){
        return 1;
    }

    // load slave address into data register
    TWDR = address;
    // start transmission of address
    TWCR = (1<<TWINT) | (1<<TWEN);
    // wait for end of transmission
    while( !(TWCR & (1<<TWINT)) )
        ;

    // check if the device has acknowledged the READ / WRITE mode
    uint8_t twst = TW_STATUS & 0xF8;
    if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ){
        return 1;
    }

    return 0;
}

//send stop signal
void TWIStop(void)
{
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
    while(TWCR & (1<<TWSTO))
        ;
}


uint8_t TWIWrite(uint8_t data)
{
    // load data into data register
    TWDR = data;
    // start transmission of data
    TWCR = (1<<TWINT) | (1<<TWEN);
    // wait for end of transmission
    while( !(TWCR & (1<<TWINT)) )
        ;

    if( (TWSR & 0xF8) != TW_MT_DATA_ACK ){
        return 1;
    }

    return 0;
}


uint8_t TWIReadACK(void)
{
    TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
    while ((TWCR & (1<<TWINT)) == 0)
        ;
    return TWDR;
}
//read byte with NACK
uint8_t TWIReadNACK(void)
{
    TWCR = (1<<TWINT)|(1<<TWEN);
    while ((TWCR & (1<<TWINT)) == 0)
        ;
    return TWDR;
}


uint8_t TWIGetStatus(void)
{
    uint8_t status;
    //mask status
    status = TWSR & 0xF8;
    return status;
}



