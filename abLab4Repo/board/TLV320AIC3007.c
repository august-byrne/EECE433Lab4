/*****************************************************************************************
 * TLV320AIC3007.c - Module for configuration and control of
 * TLV320AIC3007 Codec module.  Requires I2C.c module for communication.
 *
 *  Created on: Oct 28, 2014
 *      Author: ATC
 *      Edited by: Jacie Unpingco and Alexander Jamshedi, 03/13/2015
 *      Edited by: Todd Morton, 09/03/2015
 *                 This still needs a lot of work. Time consuming as designed.
 *****************************************************************************************
* Master Include File
*****************************************************************************************/
#include "MCUType.h"
#include "TLV320AIC3007.h"
#include "K65TWR_GPIO.h"
#include "I2C.h"

/*****************************************************************************************
* Constant CODEC register tables
*****************************************************************************************/
static const INT8U CODECDefaultCfg0[] = {
  0x00,//starting address 0
  0x00,0x00,0x00,0x91,0x20,0x1E,0x00,0x0A,0xC0,0xE0,    //Pg0, Registers 0-9
                                                        //Fs=FSref,PLL Enabled,
                                                        //FSref=48kHz w/ MCLK=12MHz
                                                        //BCLK and WCLK are Outputs,no 3D,
                                                        //Left Justified mode,24-bit
  0x01,0x01,0x50,0x00,0x00,0x00,0x00,0xFF,0xFF,0x04,    //Pg0, Registers 10-19
  0xF8,0x78,0x04,0x7C,0x78,0x80,0x00,0x00,0x00,0x00,    //Pg0, Registers 20-29
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xD0,0x00,0x00,    //Pg0, Registers 30-39
  0x80,0x00,0x8c,0x00,0x00,0x00,0x00,0x80,0x00,0x00,    //Pg0, Registers 40-49 changed reg 42 from 0x60 to 0x8c for more pop control
  0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,    //Pg0, Registers 50-59
  0x00,0x00,0x00,0x00,0x80,0x01                         //Pg0, Registers 60-65
};

static const INT8U CODECDefaultCfg1[] = {
    0x50,//starting address 80 decimal
    0x00,0x00,0x80,0x00,0x00,0x00,0x09,0x00,0x00,0x00,
    0x00,0x00,0x80,0x09
};

static const INT8U CODECDefaultCfg2[] = {
    0x65, //starting address 101 decimal
    0x00,0x02
};

static const INT8U CODECDefaultCfg3[] = {
    0x6C, //starting address 108 decimal
    0x00,0xC0
};

static const INT8U classDAmpOn[] = {
    0x49, //starting address 73 decimal (page 0)
    0x0C //0000 1100
};

static const INT8U classDAmpOff[] = {
    0x49, //starting address 73 decimal (page 0)
    0x00 //0000 0000
};

static const INT8U headPhoneOutOn[] = {
    0x33, //starting address 51 decimal (page 0)
    0x0F, //0000 1111
    0x41, //starting address 65 decimal
    0x0F
};

/********************************************************************
* Function Definitions
*********************************************************************
* CODECInit(void) - Public
*  DESCRIPTION: Configures PORTB bit 10, Resets and configures I2C
*               and CODEC.
*
*  PARAMETERS: none.
*
*  RETURN: none.
********************************************************************/
void CODECInit(void){
    /* Initialize PORTB bit 10 for the CODEC /RESET pin */
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK; /* Enable clock gate for PORTB */
    PORTB->PCR[10] |= PORT_PCR_MUX(1);
    CODECDisable();                     /*Init to Reset */
    GPIOB->PDDR |= GPIO_PIN(10);

    CODECEnable();

    I2CInit(); //init our I2C
    I2CSetSlaveAddress(0x18);
    CODECSetPage(0x00);
    CODECDefaultConfig();
    CODECHeadphoneOutOn();


}

/*********************************************************************
* CODECDefaultConfig(void) - Public
*  DESCRIPTION: Configures CODEC to default configuration.
*
*  PARAMETERS: none.
*
*  RETURN: INT8U -  0=NAK from slave or bias value invalid.
*                   1=successful transmit
********************************************************************/
INT8U CODECDefaultConfig(void){

    I2CSendStart();
    if(!I2CSendBlock((INT8U *)CODECDefaultCfg0,sizeof(CODECDefaultCfg0))){
        return 0;   //failed I2C transmit
    }
    I2CSendStop();

    I2CSendStart();
    if(!I2CSendBlock((INT8U *)CODECDefaultCfg1,sizeof(CODECDefaultCfg1))){
        return 0;   //failed I2C transmit
    }
    I2CSendStop();

    I2CSendStart();
    if(!I2CSendBlock((INT8U *)CODECDefaultCfg2,sizeof(CODECDefaultCfg2))){
        return 0;   //failed I2C transmit
    }
    I2CSendStop();

    I2CSendStart();
    if(!I2CSendBlock((INT8U *)CODECDefaultCfg3,sizeof(CODECDefaultCfg3))){
        return 0;   //failed I2C transmit
    }
    I2CSendStop();

    return 1;
}


/********************************************************************
* CODECReadRegister() - Public
*  DESCRIPTION: Reads a CODEC register
*
*  PARAMETERS: INT8U page - register page
*              INT8U raddr - register address
*
*  RETURN: INT8U -  Contents of register
********************************************************************/
INT8U CODECReadRegister(INT8U page, INT8U raddr){
    INT8U reg_in;
    (void)CODECSetPage(page);
    reg_in = I2CReadByte(raddr);
    return reg_in;
}


/********************************************************************
* CODECWriteRegister() - Public
*  DESCRIPTION: Reads a CODEC register
*
*  PARAMETERS: INT8U page - register page
*              INT8U raddr - register address
*              INT8U rval - value to write to register
*
********************************************************************/
void CODECWriteRegister(INT8U page, INT8U raddr, INT8U rval){

    (void)CODECSetPage(page);
    I2CWriteByte(raddr, rval);

}


/********************************************************************
* CODECEnableClassD(INT8U dBVolume) - Public
*  DESCRIPTION: Enables Class D amplifier and sets volume according to
*   input.
*
*  PARAMETERS: INT8U dBVolume - Class D amplifier volume in dB.  Value
*   will be rounded to nearest: 0, 6, 12, or 18 dB.  Left and right
*   channels will be equal
*
*  RETURN: INT8U -  0=NAK from slave or bias value invalid.
*                   1=successful transmit
********************************************************************/
INT8U CODECEnableClassD(INT8U dBVolume){
    INT8U i2cStatus =1;

    //force dBVolume to valid value
    if(dBVolume<=3){
        dBVolume = 0;
    }
    else if(dBVolume <=9){
        dBVolume = 6;
    }
    else if(dBVolume <=15){
        dBVolume = 12;
    }
    else{
        dBVolume = 18;
    }

    I2CSendStart();
    if(!I2CSendBlock((INT8U *)classDAmpOn,sizeof(classDAmpOn))){
        i2cStatus = 0;   //failed I2C transmit
    }
    else{
        i2cStatus = 1;
    }
    I2CSendStop();

    I2CSendStart();
    if(!I2CSendByte(dBVolume)){
        i2cStatus = 0;
    }else{
        i2cStatus = 1;
    }
    I2CSendStop();

    return i2cStatus;
}

/********************************************************************
* CODECDisableClassD(void) - Public
*  DESCRIPTION: Disables Class D Amplifier and resets volume to 0dB.
*
*  PARAMETERS: none.
*
*  RETURN: INT8U -  0=NAK from slave or bias value invalid.
*                   1=successful transmit
********************************************************************/
INT8U CODECDisableClassD(void){
    INT8U i2cStatus = 1;

    I2CSendStart();
    if(!I2CSendBlock((INT8U *)classDAmpOff,sizeof(classDAmpOff))){
        i2cStatus = 0;   //failed I2C transmit
    }
    else{
        i2cStatus = 1;
    }
    I2CSendStop();

    return i2cStatus;
}
/********************************************************************
* CODECHeadphoneOutOn(void) - Public
*  DESCRIPTION: Enables Headphone Output and sets volume according to
*   input.
*
*  PARAMETERS: none.
*   Values greater than 9 are clipped to 9.
*
*  RETURN: INT8U -  0=NAK from slave or bias value invalid.
*                   1=successful transmit
********************************************************************/
INT8U CODECHeadphoneOutOn(void){
    INT8U i2cStatus =1;

    I2CSendStart();
    if(!I2CSendBlock((INT8U *)headPhoneOutOn,sizeof(headPhoneOutOn)/2)){
        i2cStatus = 0;   //failed I2C transmit
    }
    else{
        i2cStatus = 1;
    }
    I2CSendStop();

    I2CSendStart();
    if(!I2CSendBlock((INT8U *)(headPhoneOutOn + sizeof(headPhoneOutOn)/2),sizeof(headPhoneOutOn)/2)){
        i2cStatus = i2cStatus || 0;   //failed I2C transmit
    }
    else{
        i2cStatus = 1;
    }
    I2CSendStop();

    return i2cStatus;
}

/********************************************************************
* CODECSetMicBias(INT8U bias) - Public
*
*  PARAMETERS: bias -   0x00=MICBIAS output powered down.
*                       0x01=MICBIAS output is 2.0V
*                       0x02=MICBIAS output is 2.5V
*                       0x03=MICBIAS output is connected to AVDD
*
*  RETURN: INT8U -  0=NAK from slave or bias value invalid.
*                   1=successful transmit
*
*  DESCRIPTION: Sets the MICBIAS to one of four options.
*
********************************************************************/
INT8U CODECSetMicBias(INT8U bias){

    INT8U biasData[] = {0x19, (bias<<6)};

    I2CSendStart();
    if(!I2CSendBlock(biasData,2)){
        return 0;   //failed I2C transmit
    }

    I2CSendStop();

    return 1;   //data transmit successful, ACK received.
}

/*********************************************************************
* CODECSetPage(INT8U page) - Public
*
*  PARAMETERS: page -   0=page 0
*                       1=page 1
*
*  RETURN:  INT8U -     0=NAK from slave or input out of range
*  DESCRIPTION: Sets the active page for control register access.
*
********************************************************************/
INT8U CODECSetPage(INT8U page){
    INT8U pageData[] = {0x00,page};

    I2CSendStart();
    if(!I2CSendBlock(pageData,2)){
        return 0;   //failed I2C transmit
    }
    I2CSendStop();

    return 1;
}

/*********************************************************************
* CODECEnable(void) - Public
*
*  PARAMETERS: none.
*
*  RETURN:  none.
*  DESCRIPTION: Driver active low reset pin on CODEC high.
*
********************************************************************/
void CODECEnable(void){

    //drive /RESET pin of CODEC high
    GPIOB->PSOR = GPIO_PIN(10);
}

/*********************************************************************
* CODECDisable(void) - Public
*
*  PARAMETERS: none.
*
*  RETURN:  none.
*  DESCRIPTION: Driver active low reset pin on CODEC high.
*
********************************************************************/
void CODECDisable(void){

    //drive /RESET pin of CODEC low
    GPIOB->PCOR = GPIO_PIN(10);
}

/*********************************************************************
* CODECSetSampleRate(INT8U rateCode) - Public
*
*  PARAMETERS: rateCode - 0x0 to 0xA correstponds to (if Fsref = 48kHz):
*   0000: ADC Fs = Fsref/1      (48ksps)
*   0001: ADC Fs = Fsref/1.5    (32ksps)
*   0010: ADC Fs = Fsref/2      (24ksps)
*   0011: ADC Fs = Fsref/2.5    (19.2ksps)
*   0100: ADC Fs = Fsref/3      (16ksps)
*   0101: ADC Fs = Fsref/3.5    (13.7ksps)
*   0110: ADC Fs = Fsref/4      (12ksps)
*   0111: ADC Fs = Fsref/4.5    (10.7ksps)
*   1000: ADC Fs = Fsref/5      (9.6ksps)
*   1001: ADC Fs = Fsref/5.5    (8.7ksps)
*   1010: ADC Fs = Fsref/6      (8ksps)
*
*  RETURN:  INT8U - 0=NAK from slave.  1=ACK.
*  DESCRIPTION: Sets ADC and DAC sample rates.
*  TDM, 03/31/2016
********************************************************************/
INT8U CODECSetSampleRate(INT8U rateCode){
    INT8U sampleRateData[] = {0x02,(rateCode<<4|rateCode)};   //Register 2

    I2CSendStart();
    if(!I2CSendBlock(sampleRateData,2)){
        return 0;   //failed I2C transmit
    }
    I2CSendStop();

    return 1;   //data transmit successful, ACK received.
}

/*********************************************************************
* CODECSetSampleSize(INT8U sizeCode) - Public
*
*  PARAMETERS: rateCode - 0x0 to 0x3 correstponds to:
*   0x0: 16-bit samples
*   0x1: 20-bit samples
*   0x2: 24-bit samples
*   0x3: 32-bit samples
*
*  RETURN:  INT8U - 0=NAK from slave.  1=ACK.
*  DESCRIPTION: Sets ADC and DAC sample sizes.
*  Notes: TODO: This needs to be modified so it uses read-modify-write.
*         To change the sample size the CODEC and the I2S word size
*         must be changed.
********************************************************************/
INT8U CODECSetSampleSize(INT8U sizeCode){
    INT8U sampleSizeData[] = {0x09,((0xC|sizeCode)<<4)};   //Register 9

    I2CSendStart();
    if(!I2CSendBlock(sampleSizeData,2)){
        return 0;   //failed I2C transmit
    }
    I2CSendStop();

    return 1;   //data transmit successful, ACK received.
}

/*********************************************************************
* CODECConfigPLL(void) - Public
*
*  PARAMETERS: none.
*
*  RETURN:  ITN8U - 0=NAK from slave.  1=ACK.
*  DESCRIPTION: Configures CODEC for P=1, R=1,J=8,D=5264, PLL Enabled,
*  Q=don't care.
*
********************************************************************/
INT8U CODECConfigPLL(void){
    INT8U pllData1[] = {0x03,0x91};
    INT8U pllData2[] = {0x04,0x1B};
    INT8U pllData3[] = {0x05, 0x52};
    INT8U pllData4[] = {0x06, 0x40};
    INT8U pllData5[] = {0x0B, 0x01};

    I2CSendStart();
    if(!I2CSendBlock(pllData1,2)){
        return 0;   //failed I2C transmit
    }
    I2CSendStop();

    I2CSendStart();
    if(!I2CSendBlock(pllData2,2)){
        return 0;   //failed I2C transmit
    }
    I2CSendStop();

    I2CSendStart();
    if(!I2CSendBlock(pllData3,2)){
        return 0;   //failed I2C transmit
    }
    I2CSendStop();

    I2CSendStart();
    if(!I2CSendBlock(pllData4,2)){
        return 0;   //failed I2C transmit
    }
    I2CSendStop();

    I2CSendStart();
    if(!I2CSendBlock(pllData5,2)){
        return 0;   //failed I2C transmit
    }
    I2CSendStop();

    return 1;   //data transmit successful, ACK received.
}

/*********************************************************************
* CODECSetDataPath(void) - Public
*
*  PARAMETERS: none.
*
*  RETURN:  ITN8U - 0=NAK from slave.  1=ACK.
*  DESCRIPTION: Sets data path for Left input to play to left output.
*  Right input to play to right output.
*
********************************************************************/
INT8U CODECSetDataPath(void){
    INT8U pathData[] = {0x07, 0x0A};

    I2CSendStart();
    if(!I2CSendBlock(pathData,2)){
        return 0;   //failed I2C transmit
    }
    I2CSendStop();

    return 1;
}

/*********************************************************************
* CODECConfigASI(void) - Public
*
*  PARAMETERS: none.
*
*  RETURN:  ITN8U - 0=NAK from slave.  1=ACK.
*  DESCRIPTION: Configures Audio Serial Interface for BCLK, WCLK as outputs,
*  no 3-state output, disabled 3-D effects
*
********************************************************************/
INT8U CODECConfigASI(void){
    INT8U asiData1[] = {0x08, 0x00};
    INT8U asiData2[] = {0x09, 0x20};
    INT8U asiData3[] = {0x0A, 0x01};

    I2CSendStart();
    if(!I2CSendBlock(asiData1,2)){
        return 0;   //failed I2C transmit
    }
    I2CSendStop();

    I2CSendStart();
    if(!I2CSendBlock(asiData2,2)){
        return 0;   //failed I2C transmit
    }
    I2CSendStop();

    I2CSendStart();
    if(!I2CSendBlock(asiData3,2)){
        return 0;   //failed I2C transmit
    }
    I2CSendStop();

    return 1;
}

/*********************************************************************
* CODECConfigFilter(void) - Public
*
*  PARAMETERS: none.
*
*  RETURN:  ITN8U - 0=NAK from slave.  1=ACK.
*  DESCRIPTION: Configures Digital Filter.  Currently all disabled.
*
********************************************************************/
INT8U CODECConfigFilter(void){
    INT8U filterData[] = {0x0C, 0x00};

    I2CSendStart();
    if(!I2CSendBlock(filterData,2)){
        return 0;   //failed I2C transmit
    }
    I2CSendStop();

    return 1;
}

/*********************************************************************
* CODECConfigHeadset(void) - Public
*
*  PARAMETERS: none.
*
*  RETURN:  ITN8U - 0=NAK from slave.  1=ACK.
*  DESCRIPTION: Configures Headset detection to be enabled.
*
********************************************************************/
INT8U CODECConfigHeadset(void){
    INT8U headsetData[] = {0x0D, 0xA0};

    I2CSendStart();
    if(!I2CSendBlock(headsetData,2)){
        return 0;   //failed I2C transmit
    }
    I2CSendStop();

    return 1;
}

/*********************************************************************
* CODECConfigHeadsetDrive(void) - Public
*
*  PARAMETERS: none.
*
*  RETURN:  ITN8U - 0=NAK from slave.  1=ACK.
*  DESCRIPTION: Configures Headset driver mode.
*
********************************************************************/
INT8U CODECConfigHeadsetDrive(void){
    INT8U headsetDriveData[] = {0x0E, 0x00};

    I2CSendStart();
    if(!I2CSendBlock(headsetDriveData,2)){
        return 0;   //failed I2C transmit
    }
    I2CSendStop();

    return 1;
}

/*********************************************************************
* CODECConfigLeftADCGain(void) - Public
*
*  PARAMETERS: none.
*
*  RETURN:  ITN8U - 0=NAK from slave.  1=ACK.
*  DESCRIPTION: Configures left ADC gain for 0dB.
*
********************************************************************/
INT8U CODECConfigLeftADCGain(void){
    INT8U gainLData[] = {0x0F, 0x00};

    I2CSendStart();
    if(!I2CSendBlock(gainLData,2)){
        return 0;   //failed I2C transmit
    }
    I2CSendStop();

    return 1;
}

/*********************************************************************
* CODECConfigRightADCGain(void) - Public
*
*  PARAMETERS: none.
*
*  RETURN:  ITN8U - 0=NAK from slave.  1=ACK.
*  DESCRIPTION: Configures right ADC gain for 0dB.
*
********************************************************************/
INT8U CODECConfigRightADCGain(void){
    INT8U gainRData[] = {0x10, 0x00};

    I2CSendStart();
    if(!I2CSendBlock(gainRData,2)){
        return 0;   //failed I2C transmit
    }
    I2CSendStop();

    return 1;
}

/*********************************************************************
* CODECConfigMIC3LRGain(void) - Public
*
*  PARAMETERS: none.
*
*  RETURN:  ITN8U - 0=NAK from slave.  1=ACK.
*  DESCRIPTION: Configures MIC3LR gain for 0dB.
*
********************************************************************/
INT8U CODECConfigMIC3LRGain(void){
    //register 17, input level control gain is 0 dB
    INT8U mic3LRData1[] = {0x11, 0x00};
    //register 18, input level control gain is 0 dB
    INT8U mic3LRData2[] = {0x12, 0x00};

    I2CSendStart();
    if(!I2CSendBlock(mic3LRData1,2)){
        return 0;   //failed I2C transmit
    }
    I2CSendStop();

    I2CSendStart();
    if(!I2CSendBlock(mic3LRData2,2)){
        return 0;   //failed I2C transmit
    }
    I2CSendStop();

    return 1;
}

/*********************************************************************
* CODECConfigLine1L(void) - Public
*
*  PARAMETERS: none.
*
*  RETURN:  ITN8U - 0=NAK from slave.  1=ACK.
*  DESCRIPTION: Configures Line1L for single ended 0dB gain.
*
********************************************************************/
INT8U CODECConfigLine1L(void){
    //register 19, left ADC channel is on, input level control gain is 0 dB
    INT8U line1LData[] = {0x13, 0x04};
    //register 24, not connected to right ADC PGA
    INT8U line1LtoRightData[] = {0x18, 0x78};

    I2CSendStart();
    if(!I2CSendBlock(line1LData,2)){
        return 0;   //failed I2C transmit
    }
    I2CSendStop();

    I2CSendStart();
    if(!I2CSendBlock(line1LtoRightData,2)){
        return 0;   //failed I2C transmit
    }
    I2CSendStop();

    return 1;
}

/*********************************************************************
* CODECConfigLine2L(void) - Public
*
*  PARAMETERS: none.
*
*  RETURN:  ITN8U - 0=NAK from slave.  1=ACK.
*  DESCRIPTION: Configures Line2L to be fully differential, 0dB gain.
*
********************************************************************/
INT8U CODECConfigLine2L(void){
    INT8U line2LData[] = {0x14, 0x80};

    I2CSendStart();
    if(!I2CSendBlock(line2LData,2)){
        return 0;   //failed I2C transmit
    }
    I2CSendStop();

    return 1;
}

/*********************************************************************
* CODECConfigLine1R(void) - Public
*
*  PARAMETERS: none.
*
*  RETURN:  ITN8U - 0=NAK from slave.  1=ACK.
*  DESCRIPTION: Configures Line1R for single ended no gain to left ADC.
*
********************************************************************/
INT8U CODECConfigLine1R(void){
    //register 21, not connected to left ADC PGA
    INT8U line1RtoLeftData[] = {0x15, 0x78};
    //register 22, right ADC channel is on, input level control gain 0 dB
    INT8U line1RtoRightData[] = {0x16, 0x04};

    I2CSendStart();
    if(!I2CSendBlock(line1RtoLeftData,2)){
        return 0;   //failed I2C transmit
    }
    I2CSendStop();

    I2CSendStart();
    if(!I2CSendBlock(line1RtoRightData,2)){
        return 0;   //failed I2C transmit
    }
    I2CSendStop();

    return 1;
}

/*********************************************************************
* CODECConfigLine2R(void) - Public
*
*  PARAMETERS: none.
*
*  RETURN:  ITN8U - 0=NAK from slave.  1=ACK.
*  DESCRIPTION: Configures Line1R for single ended no gain to left ADC.
*
********************************************************************/
INT8U CODECConfigLine2R(void){
    INT8U line2RtoRightData[] = {0x17, 0x80};

    I2CSendStart();
    if(!I2CSendBlock(line2RtoRightData,2)){
        return 0;   //failed I2C transmit
    }
    I2CSendStop();

    return 1;
}

/*********************************************************************
* CODECConfigLeftAGC(void) - Public
*
*  PARAMETERS: none.
*
*  RETURN:  ITN8U - 0=NAK from slave.  1=ACK.
*  DESCRIPTION: Currently just turns off Left AGC.
*
********************************************************************/
INT8U CODECConfigLeftAGC(void){
    INT8U leftAGCData[] = {0x1A, 0x00};

    I2CSendStart();
    if(!I2CSendBlock(leftAGCData,2)){
        return 0;   //failed I2C transmit
    }
    I2CSendStop();

    return 1;
}

/*********************************************************************
* CODECConfigRightAGC(void) - Public
*
*  PARAMETERS: none.
*
*  RETURN:  ITN8U - 0=NAK from slave.  1=ACK.
*  DESCRIPTION: Currently just turns off Left AGC.
*
********************************************************************/
INT8U CODECConfigRightAGC(void){
    INT8U rightAGCData[] = {0x1D, 0x00};

    I2CSendStart();
    if(!I2CSendBlock(rightAGCData,2)){
        return 0;   //failed I2C transmit
    }
    I2CSendStop();

    return 1;
}

/*********************************************************************
* CODECConfigDAC(void) - Public
*
*  PARAMETERS: none.
*
*  RETURN:  ITN8U - 0=NAK from slave.  1=ACK.
*  DESCRIPTION: Powers on DACs, enables HPCOM as constant VCM output
*
********************************************************************/
INT8U CODECConfigDAC(void){
    INT8U DACData1[] = {0x25, 0xD0}; //11010000
    INT8U DACData2[] = {0x29, 0xD0}; //11010000

    I2CSendStart();
    if(!I2CSendBlock(DACData1,2)){
        return 0;   //failed I2C transmit
    }
    I2CSendStop();

    I2CSendStart();
    if(!I2CSendBlock(DACData2,2)){
        return 0;   //failed I2C transmit
    }
    I2CSendStop();

    return 1;
}

/*********************************************************************
* CODECConfigHighPower(void) - Public
*
*  PARAMETERS: none.
*
*  RETURN:  ITN8U - 0=NAK from slave.  1=ACK.
*  DESCRIPTION: Powers on DACs, enables HPCOM as constant VCM output
*
********************************************************************/
INT8U CODECConfigHighPower(void){
    INT8U powerDriveData[] = {0x26, 0x02}; //0000010

    I2CSendStart();
    if(!I2CSendBlock(powerDriveData,2)){
        return 0;   //failed I2C transmit
    }
    I2CSendStop();

    return 1;
}

/*********************************************************************
* CODECConfigLeftDACVolume(void) - Public
*
*  PARAMETERS: none.
*
*  RETURN:  ITN8U - 0=NAK from slave.  1=ACK.
*  DESCRIPTION: Powers on DACs, enables HPCOM as constant VCM output
*
********************************************************************/
INT8U CODECConfigLeftDACVolume(void){
    INT8U LdacVolData[] = {0x2B, 0x00}; //00000000

    I2CSendStart();
    if(!I2CSendBlock(LdacVolData,2)){
        return 0;   //failed I2C transmit
    }
    I2CSendStop();

    return 1;
}

/*********************************************************************
* CODECConfigRightDACVolume(void) - Public
*
*  PARAMETERS: none.
*
*  RETURN:  ITN8U - 0=NAK from slave.  1=ACK.
*  DESCRIPTION: Powers on DACs, enables HPCOM as constant VCM output
*
********************************************************************/
INT8U CODECConfigRightDACVolume(void){
    INT8U RdacVolData[] = {0x2C, 0x00}; //00000000

    I2CSendStart();
    if(!I2CSendBlock(RdacVolData,2)){
        return 0;   //failed I2C transmit
    }
    I2CSendStop();

    return 1;
}

/*********************************************************************
* CODECConfigHPLOUT(void) - Public
*
*  PARAMETERS: none.
*
*  RETURN:  ITN8U - 0=NAK from slave.  1=ACK.
*  DESCRIPTION: Powers on DACs, enables HPCOM as constant VCM output
*
********************************************************************/
INT8U CODECConfigHPLOUT(void){
    INT8U HPLOUTData1[] = {0x2E, 0x80}; //1000000
    INT8U HPLOUTData2[] = {0x2F, 0x80}; //1000000
    INT8U HPLOUTData3[] = {0x33, 0x0D}; //00001101

    I2CSendStart();
    if(!I2CSendBlock(HPLOUTData1,2)){  //connect PGA_L to HPLOUT
        return 0;   //failed I2C transmit
    }
    I2CSendStop();

    I2CSendStart();
    if(!I2CSendBlock(HPLOUTData2,2)){  //connect DAC_L1 to HPLOUT
        return 0;   //failed I2C transmit
    }
    I2CSendStop();

    I2CSendStart();
    if(!I2CSendBlock(HPLOUTData3,2)){  //connect DAC_L1 to HPLOUT
        return 0;   //failed I2C transmit
    }
    I2CSendStop();

    return 1;
}

/*********************************************************************
* CODECConfigHPROUT(void) - Public
*
*  PARAMETERS: none.
*
*  RETURN:  ITN8U - 0=NAK from slave.  1=ACK.
*  DESCRIPTION: Powers on DACs, enables HPCOM as constant VCM output
*
********************************************************************/
INT8U CODECConfigHPROUT(void){
    INT8U HPROUTData1[] = {0x3F, 0x80}; //10000000
    INT8U HPROUTData2[] = {0x40, 0x80}; //1000000
    INT8U HPROUTData3[] = {0x41, 0x0D}; //00001101

    I2CSendStart();
    if(!I2CSendBlock(HPROUTData1,2)){  //connect PGA_L to HPLOUT
        return 0;   //failed I2C transmit
    }
    I2CSendStop();

    I2CSendStart();
    if(!I2CSendBlock(HPROUTData2,2)){  //connect DAC_L1 to HPLOUT
        return 0;   //failed I2C transmit
    }
    I2CSendStop();

    I2CSendStart();
    if(!I2CSendBlock(HPROUTData3,2)){  //connect DAC_L1 to HPLOUT
        return 0;   //failed I2C transmit
    }
    I2CSendStop();

    return 1;
}

/*********************************************************************
* CODECConfigLeftLOP(void) - Public
*
*  PARAMETERS: none.
*
*  RETURN:  ITN8U - 0=NAK from slave.  1=ACK.
*  DESCRIPTION: Powers on DACs, enables HPCOM as constant VCM output
*
********************************************************************/
INT8U CODECConfigLeftLOP(void){
    INT8U LeftLOPData1[] = {0x56, 0x08}; //00001000

    I2CSendStart();
    if(!I2CSendBlock(LeftLOPData1,2)){ //connect PGA_L to HPLOUT
        return 0;   //failed I2C transmit
    }
    I2CSendStop();

    return 1;
}

/*********************************************************************
* CODECConfigRightLOP(void) - Public
*
*  PARAMETERS: none.
*
*  RETURN:  ITN8U - 0=NAK from slave.  1=ACK.
*  DESCRIPTION: Powers on DACs, enables HPCOM as constant VCM output
*
********************************************************************/
INT8U CODECConfigRightLOP(void){
    INT8U RightLOPData1[] = {0x5D, 0x08}; //00001000

    I2CSendStart();
    if(!I2CSendBlock(RightLOPData1,2)){    //connect PGA_L to HPLOUT
        return 0;   //failed I2C transmit
    }
    I2CSendStop();

    return 1;
}
