/*****************************************************************************************************
* AppDSP.h
*
* 02/10/2017 Todd Morton
*****************************************************************************************************/

/*****************************************************************************************************
* Module definition against multiple inclusion
*****************************************************************************************************/
#ifndef  SAMPLE_PROC_PRESENT
#define  SAMPLE_PROC_PRESENT

/*****************************************************************************************************
* DSP configuration constants.
* Here a buffer is the complete data object, which may be comprised of multiple blocks. For example,
* when using a ping-pong buffer, there are two blocks.
*****************************************************************************************************/
#define DSP_NUM_BLOCKS                  2
#define DSP_SAMPLES_PER_BLOCK           512
#define DSP_BUFFER_BYTES_PER_SAMPLE     4
#define DSP_NUM_IN_CHANNELS             2
#define DSP_NUM_OUT_CHANNELS            2
#define DSP_LEFT_CH                     0
#define DSP_RIGHT_CH                    1

/*****************************************************************************************************
* DSP global sample blocks, typedef
*****************************************************************************************************/
typedef struct{
    q31_t samples[DSP_SAMPLES_PER_BLOCK];
} DSP_BLOCK_T;

//Sample size codes
#define DSP_SSIZE_CODE_16BIT        CODEC_SSIZE_CODE_16BIT
#define DSP_SSIZE_CODE_20BIT        CODEC_SSIZE_CODE_20BIT
#define DSP_SSIZE_CODE_24BIT        CODEC_SSIZE_CODE_24BIT
#define DSP_SSIZE_CODE_32BIT        CODEC_SSIZE_CODE_32BIT

//Sample rate codes
#define DSP_SRATE_CODE_48K     CODEC_SRATE_CODE_48K
#define DSP_SRATE_CODE_32K     CODEC_SRATE_CODE_32K
#define DSP_SRATE_CODE_24K     CODEC_SRATE_CODE_24K
#define DSP_SRATE_CODE_19P2K   CODEC_SRATE_CODE_19P2K
#define DSP_SRATE_CODE_16K     CODEC_SRATE_CODE_16K
#define DSP_SRATE_CODE_13P7K   CODEC_SRATE_CODE_13P7K
#define DSP_SRATE_CODE_12K     CODEC_SRATE_CODE_12K
#define DSP_SRATE_CODE_10P7K   CODEC_SRATE_CODE_10P7K
#define DSP_SRATE_CODE_9P6K    CODEC_SRATE_CODE_9P6K
#define DSP_SRATE_CODE_8P7K    CODEC_SRATE_CODE_8P7K
#define DSP_SRATE_CODE_8K      CODEC_SRATE_CODE_8K

typedef struct{
    INT16U srate;
    INT8U ssize;
} DSP_PARAMS_T;

typedef enum{LEFT_IN, RIGHT_IN, LEFT_OUT, RIGHT_OUT} BUFF_ID_T;
/*****************************************************************************************************
* Declaration of project wide FUNCTIONS
*****************************************************************************************************/
void DSPInit(void);
void DSPSampleSizeSet(INT8U ssize_code);
void DSPSampleRateSet(INT8U rate_code);
INT8U DSPSampleSizeGet(void);
INT16U DSPSampleRateGet(void);
void DSPStartReq(void);
void DSPStopReq(void);
void DSPStopFullPend(OS_TICK tout, OS_ERR *os_err_ptr);
INT32S *DSPBufferGet(BUFF_ID_T buff_id);

#endif
