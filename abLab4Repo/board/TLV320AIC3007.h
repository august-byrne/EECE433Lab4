/*******************************************************************************/
/* TLV320AIC3007.h - Project Header file for I2C communications
* ATC 10/28/14*/
/*******************************************************************************/
// Public Function Prototypes
/*******************************************************************************/
void CODECInit(void);
INT8U CODECDefaultConfig(void);
INT8U CODECReadRegister(INT8U page, INT8U raddr);
void CODECWriteRegister(INT8U page, INT8U raddr, INT8U rval);

INT8U CODECEnableClassD(INT8U dBVolume);
INT8U CODECDisableClassD(void);
INT8U CODECHeadphoneOutOn(void);
INT8U CODECSetSampleRate(INT8U rateCode);
INT8U CODECSetSampleSize(INT8U sizeCode);
void CODECEnable(void);
void CODECDisable(void);
// TODO: The following functions have not been completed.
INT8U CODECSetMicBias(INT8U bias);
INT8U CODECSetPage(INT8U page);
INT8U CODECConfigPLL(void);
INT8U CODECSetDataPath(void);
INT8U CODECConfigASI(void);
INT8U CODECConfigFilter(void);
INT8U CODECConfigHeadset(void);
INT8U CODECConfigHeadsetDrive(void);
INT8U CODECConfigLeftADCGain(void);
INT8U CODECConfigRightADCGain(void);
INT8U CODECConfigMIC3LRGain(void);
INT8U CODECConfigLine1L(void);
INT8U CODECConfigLine2L(void);
INT8U CODECConfigLine1R(void);
INT8U CODECConfigLine2R(void);
INT8U CODECConfigLeftAGC(void);
INT8U CODECConfigRightAGC(void);
INT8U CODECConfigDAC(void);
INT8U CODECConfigHighPower(void);
INT8U CODECConfigLeftDACVolume(void);
INT8U CODECConfigRightDACVolume(void);
INT8U CODECConfigHPLOUT(void);
INT8U CODECConfigHPROUT(void);
INT8U CODECConfigRightLOP(void);
INT8U CODECConfigLeftLOP(void);
/*******************************************************************************/
//Sample rate codes assuming FSref is 48kHz

#define CODEC_SRATE_CODE_48K            0x0
#define CODEC_SRATE_CODE_32K            0x1
#define CODEC_SRATE_CODE_24K            0x2
#define CODEC_SRATE_CODE_19P2K          0x3
#define CODEC_SRATE_CODE_16K            0x4
#define CODEC_SRATE_CODE_13P7K          0x5
#define CODEC_SRATE_CODE_12K            0x6
#define CODEC_SRATE_CODE_10P7K          0x7
#define CODEC_SRATE_CODE_9P6K           0x8
#define CODEC_SRATE_CODE_8P7K           0x9
#define CODEC_SRATE_CODE_8K             0xa

//Sample size codes
#define CODEC_SSIZE_CODE_16BIT            0x0
#define CODEC_SSIZE_CODE_20BIT            0x1
#define CODEC_SSIZE_CODE_24BIT            0x2
#define CODEC_SSIZE_CODE_32BIT            0x3

