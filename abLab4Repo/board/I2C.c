/*********************************************************************************
 * I2C.c - I2C module is written for I2C0 assuming physical
 * connection to PTE19(SCL) and PTE18(SDA). Configured for SCL
 * of ~100kHz, 7-bit addressing.
 *
 *  Created on: Oct 13, 2014
 *      Author: ATC
 *
 *      Edited by: Korey Adams, Alexander Jamshedi 3/19/2015
 * Major modifications, added register read function, Todd Morton, 03/29/2020
**********************************************************************************
* Master Include File
*********************************************************************************/
#include "MCUType.h"
#include "I2C.h"
/*********************************************************************************
* Defines
*********************************************************************************/
#define WR 0x00
#define RD 0x01
/*********************************************************************************
* Private Resources
*********************************************************************************/
static INT8U slaveAddress = 0x00;
static INT8U i2cRd(void);
static void i2cWr(INT8U dout);
static INT8U i2cGetSlaveAddress(void);
void i2cSendRepeatedStart(void);
/*********************************************************************************
* Function Definitions
**********************************************************************************
* I2CInit(void) - Public
*
*  PARAMETERS: none.
*
*  DESCRIPTION: Initializes I2C0 for Master mode, 7-bit addressing,
*   uses PTE18 and 19.
*
*********************************************************************************/
void I2CInit(void){
    SIM->SCGC4 |= SIM_SCGC4_I2C0_MASK;   //enable I2C0 clock.  SIM_SCGC4: IIC0=1
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK; /* Enable clock gate for PORTE */
//todo: I don't think the ISF bit should be set.
    /* PORTE_PCR18: ISF=0,MUX=4 */
    PORTE->PCR[18] = PORT_PCR_ISF_MASK|PORT_PCR_ODE_MASK|PORT_PCR_MUX(0x04);

    /* PORTE_PCR19: ISF=0,MUX=4 */
    PORTE->PCR[19] = PORT_PCR_ISF_MASK|PORT_PCR_ODE_MASK|PORT_PCR_MUX(0x04);
    I2C0->F = 0x9C;          //set baud rate
    I2C0->C1 = 0x80;         //enable I2C0
}

/*********************************************************************
* I2CDeInit(void) - Public
*
*  PARAMETERS: none.
*
*  DESCRIPTION: Disables I2C0 and turns off I2C0 clock.
*
********************************************************************/
void I2CDeInit(void){
    I2C0->C1 &= ~0x80;   //disabled I2C0
    SIM->SCGC4 &= ~SIM_SCGC4_I2C0_MASK;  //disable I2C0 clock
}

/*********************************************************************
* I2CSendStart(void) - Public
*
*  PARAMETERS: none.
*
*  DESCRIPTION: Sends start command on I2C Bus.
*
********************************************************************/
void I2CSendStart(void){
    while(I2C0->S & 0x20){}   //wait for bus to become idle
    I2C0->C1 |= I2C_C1_TX_MASK;  //set TX mode
    I2C0->C1 |= I2C_C1_MST_MASK; //set Master mode
}

/*********************************************************************
* I2CSendStop(void) - Public
*
*  PARAMETERS: none.
*
*  DESCRIPTION: Sends stop command on I2C Bus and switches to
*  slave mode and rx mode.
*
********************************************************************/
void I2CSendStop(void){
    while(!(I2C0->S & 0x80)){}    //wait for transfer complete flag to set
    I2C0->C1 &= ~(I2C_C1_MST_MASK);      //set Slave mode
    I2C0->C1 &= ~(I2C_C1_TX_MASK);       //set RX mode
}

/*********************************************************************
* i2cSendRepeatedStart(void) - Public
*
*  PARAMETERS: none.
*
*  DESCRIPTION: Sends repeated start command on I2C Bus.
*
********************************************************************/
void i2cSendRepeatedStart(void){
    I2C0->C1 |= 0x04;    //send repeated start
}

/*********************************************************************
* I2CSetSlaveAddress(INT8U address) - Public
*
*  PARAMETERS: address - address of slave to be addressed.
*
*  DESCRIPTION: Sets slave address variable.  This function must be
*   called once prior to communicating with a slave.
*
********************************************************************/
void I2CSetSlaveAddress(INT8U address){
    slaveAddress = address;
}

/*********************************************************************
* INT8U i2cGetSlaveAddress(void) - Public
*
*  PARAMETERS: none.
*
*  RETURN: INT8U - slave address.
*
*  DESCRIPTION: Returns current slave address.
*
********************************************************************/
static INT8U i2cGetSlaveAddress(void){
    return slaveAddress;
}

/*********************************************************************
* INT8U I2CSendBlock(INT32U *dataToSendPtr,INT8U size) - Public
*
*  PARAMETERS: *dataToSendPtr - pointer to data to be sent.
*               size - number of bytes to be sent.
*  RETURN: error code.  returns 1 if transmit was successful, 0 if failed.
*
*  DESCRIPTION: Sends slave address and a series of
*  8-bit bytes to the slave.  User must have previously set slave
*  address using I2CSetSlaveAddress(). Function returns 0 if any
*  bytes are not acknowledged by slave.  This is a blocking routine.
*  User is responsible for generating start and stop commands.
*
********************************************************************/
INT8U I2CSendBlock(INT8U *dataToSendPtr,INT8U size){
    INT8U i;
    while(!(I2C0->S & 0x80)){};    //wait for bus to be free.

    I2C0->D = (i2cGetSlaveAddress()<<1); //send slave address
    for(i=0;i<size;i++){
        while(!(I2C0->S & I2C_S_IICIF_MASK)); //wait for flag
        I2C0->S |= I2C_S_IICIF_MASK; // clear flag

        if(I2C0->S & I2C_S_RXAK_MASK){
            //RX NAK'd
            return 0;
        }
        I2C0->D = dataToSendPtr[i];  //send data byte
    }
    while(!(I2C0->S & I2C_S_IICIF_MASK)); //wait for flag
    I2C0->S |= I2C_S_IICIF_MASK; // clear flag
    if(I2C0->S & I2C_S_RXAK_MASK){
        //RX NAK'd
        return 0;
    }
    return 1;
}

/*********************************************************************
* INT8U I2CSendByte(INT8U data) - Public
*
*  PARAMETERS: INT8U data - 8-bit byte to send.
*  RETURN: error code.  returns 1 if transmit was successful, 0 if failed.
*
*  DESCRIPTION: Sends slave address and a single
*  8-bit byte to the slave.  User must have previously set slave
*  address using I2CSetSlaveAddress(). Function returns 0 if any
*  bytes are not acknowledged by slave.  This is a blocking routine.
*  User is responsible for generating start and stop commands.
*
********************************************************************/
INT8U I2CSendByte(INT8U data){
    I2C0->D = (i2cGetSlaveAddress()<<1); //send slave address
    while(!(I2C0->S & I2C_S_IICIF_MASK)); //wait for flag
    I2C0->S |= I2C_S_IICIF_MASK; // clear flag

    if(I2C0->S & I2C_S_RXAK_MASK){
        //RX NAK'd
        return 0;
    }
    I2C0->D = data;  //send data byte
    while(!(I2C0->S & I2C_S_IICIF_MASK)); //wait for flag
    I2C0->S |= I2C_S_IICIF_MASK; // clear flag
    if(I2C0->S & I2C_S_RXAK_MASK){
        //RX NAK'd
        return 0;
    }
    return 1;
}

/****************************************************************************************
* INT8U I2CReadByte(INT8U reg) - Public
*
*  PARAMETERS: INT8U reg - 8-bit register address to read.
*  RETURN: contents of register.
*
*  DESCRIPTION: Sends slave address and 8-bit register address to the slave. User must
*  have previously set slave address using I2CSetSlaveAddress().
*  Function returns the contents of the register.
*
****************************************************************************************/
INT8U I2CReadByte(INT8U reg){
    INT8U rval;
    I2CSendStart();                         /* Create I2C start                        */
    i2cWr((i2cGetSlaveAddress()<<1)|WR);    /* Send slave address & W/R' bit           */
    i2cWr(reg);                             /* Send register address                   */
    I2C0->C1 |= I2C_C1_RSTA_MASK;           /* Repeated Start                          */
    i2cWr((i2cGetSlaveAddress()<<1)|RD);    /* Send slave address & W/R' bit           */
    rval = i2cRd();                         /* Send to read return value               */
    I2CSendStop();                          /* Create I2C stop                         */
    return rval;
}
/****************************************************************************************
* void I2CWriteByte(INT8U reg, INT8U rval) - Public
*
*  PARAMETERS: INT8U reg - 8-bit register address to write.
*              INT8U rval - value to write to byte.
*
*  DESCRIPTION: Sends slave address and 8-bit register address to the slave. User must
*  have previously set slave address using I2CSetSlaveAddress().
*  Function then writes rval to the register, reg.
*
****************************************************************************************/
void I2CWriteByte(INT8U reg, INT8U rval){

    I2CSendStart();                         /* Create I2C start                        */
    i2cWr((i2cGetSlaveAddress()<<1)|WR);    /* Send slave address & W/R' bit           */
    i2cWr(reg);                           /* Send register address                   */
    i2cWr(rval);                            /* Send register value                     */
    I2CSendStop();                          /* Create I2C stop                         */
}
/****************************************************************************************
* i2cWr - Write one byte to I2C. Blocks until byte Xmit is complete
* Parameters:
*   dout is the data/address to send
****************************************************************************************/
static void i2cWr(INT8U dout){
    I2C0->D = dout;                              /* Send data/address                   */
    while((I2C0->S & I2C_S_IICIF_MASK) == 0) {}  /* Wait for completion                 */
    I2C0->S |= I2C_S_IICIF(1);                 /* Clear IICIF flag                    */
}

/****************************************************************************************
* i2cRd - Read one byte from I2C. Blocks until byte reception is complete
* Parameters:
*   Return value is the data returned from the MMA8451
****************************************************************************************/
static INT8U i2cRd(void){
    INT8U din;
    I2C0->C1 &= (INT8U)(~I2C_C1_TX_MASK);               /*Set to master receive mode           */
    I2C0->C1 |= I2C_C1_TXAK_MASK;                /*Set to no ack on read                */
    din = I2C0->D;                               /*Dummy read to generate clock cycles  */
    while((I2C0->S & I2C_S_IICIF_MASK) == 0) {}  /* Wait for completion                 */
    I2C0->S |= I2C_S_IICIF(1);                 /* Clear IICIF flag                    */
    I2CSendStop();                                  /* Send Stop                           */
    din = I2C0->D;                               /* Read data that was clocked in       */
    return din;
}

