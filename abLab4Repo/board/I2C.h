/****************************************************************************************
 * I2C.c - I2C module is written for I2C0 assuming physical
 * connection to PTE19(SCL) and PTE18(SDA). Configured for SCL
 * of ~100kHz, 7-bit addressing.
 *
 *  Created on: Oct 13, 2014
 *      Author: ATC
 *
 * Edited by: Korey Adams, Alexander Jamshedi 3/19/2015
 * Major modifications, added register read function, Todd Morton, 03/29/2020
*****************************************************************************************
* Public Function Prototypes
****************************************************************************************/
void I2CInit(void);
void I2CDeInit(void);
void I2CSetSlaveAddress(INT8U address);
void I2CSendStart(void);
void I2CSendStop(void);
INT8U I2CSendBlock(INT8U *dataToSendPtr,INT8U size);
INT8U I2CSendByte(INT8U data);
void I2CWriteByte(INT8U reg, INT8U rval);
INT8U I2CReadByte(INT8U raddr);
/***************************************************************************************/
