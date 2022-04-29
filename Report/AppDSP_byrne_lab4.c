
/*******************************************************************************************
* AppDSP.c
* This is an example of one data processing task that does some real-time digital processing.
* Edits for eece433 lab 4 were done by August Byrne.
*
* 02/10/2017 Todd Morton
* 04/03/2019 Todd Morton
* 05/14/2021 August Byrne
*******************************************************************************************/
/******************************************************************************************
* Include files
*******************************************************************************************/
#include "MCUType.h"
#include "app_cfg.h"
#include "os.h"
#include "I2S.h"
#include "TLV320AIC3007.h"
#include "K65TWR_GPIO.h"
#include "AppDSP.h"
#include "K65DMA.h"
/*****************************************************************************************************
* Defined constants for processing
*****************************************************************************************************/
#define FFT_LENGTH      512
#define NUM_TAPS        43
//Supported Lengths: 32, 64, 128, 256, 512, 1024, 2048
                                //Must be <= DSP_SAMPLES_PER_BLOCK unless zero padded
#define SAMPLE_RATE_HZ  48000
//#define FREQ_BIN_SIZE   SAMPLE_RATE_HZ/FFT_LENGTH
/******************************************************************************************/
static DSP_BLOCK_T dspInBuffer[DSP_NUM_IN_CHANNELS][DSP_NUM_BLOCKS];
static DSP_BLOCK_T dspOutBuffer[DSP_NUM_OUT_CHANNELS][DSP_NUM_BLOCKS];
static INT8U dspStopReqFlag = 0;
static OS_SEM dspFullStop;

static float32_t firCoeffF32[NUM_TAPS];

/*****************************************************************************************************
* Public Function Prototypes
*****************************************************************************************************/
//FIR variables
static arm_fir_instance_q31 FirLeft;
static arm_fir_instance_q31 FirRight;
static q31_t firLeftStateq31[NUM_TAPS + DSP_SAMPLES_PER_BLOCK - 1];
static q31_t firRightStateq31[NUM_TAPS + DSP_SAMPLES_PER_BLOCK - 1];
static q31_t firCoeffQ31[NUM_TAPS];

static float32_t cosineVals[DSP_SAMPLES_PER_BLOCK];
static float32_t BufferLeft[DSP_SAMPLES_PER_BLOCK];
static float32_t BufferRight[DSP_SAMPLES_PER_BLOCK];

static float32_t ModBufferLeft[DSP_SAMPLES_PER_BLOCK];
static float32_t ModBufferRight[DSP_SAMPLES_PER_BLOCK];
static float32_t DemodBufferLeft[DSP_SAMPLES_PER_BLOCK];
static float32_t DemodBufferRight[DSP_SAMPLES_PER_BLOCK];

//define FFT instances
static arm_rfft_instance_q31 arm_rfft_sR_q31_len128Left;
static arm_rfft_instance_q31 arm_rfft_sR_q31_len128Right;
/*******************************************************************************************
* Private Function Prototypes
*******************************************************************************************/
static void  dspTask(void *p_arg);
static CPU_STK dspTaskStk[APP_CFG_DSP_TASK_STK_SIZE];
static OS_TCB dspTaskTCB;
static DSP_PARAMS_T dspParams;
static const INT8U dspCodeToSize[4] = {16,20,24,32};
static const INT16U dspCodeToRate[11] = {48000,32000,24000,19200,16000,13700,
                                         12000,10700,9600,8700,8000};
/*******************************************************************************************
* DSPInit()- Initializes all dsp requirements - CODEC,I2S,DMA, and sets initial sample rate
*            and sample size.
*******************************************************************************************/
void DSPInit(void){
    OS_ERR os_err;
    float32_t Fc = 4000;
    float32_t V1;
    int Q = (NUM_TAPS-1)/2;

    for(int i=0;i<DSP_SAMPLES_PER_BLOCK;i++){
    	cosineVals[i] = arm_cos_f32(2*PI*i*20000/48000);
    }

    //FIR Coefficients
	V1 = Fc/(SAMPLE_RATE_HZ/2);
	firCoeffF32[Q] = V1;
    for(int i=0;i<Q;i++){
    	firCoeffF32[i] = sin((Q-i)*PI*V1)/((Q-i)*PI);
    	firCoeffF32[NUM_TAPS-1-i] =  sin((Q-i)*PI*V1)/((Q-i)*PI);
    }

    //initialize fir filter instance
    arm_fir_init_q31(&FirLeft, NUM_TAPS, (q31_t *)&firCoeffQ31[0], &firLeftStateq31[0], DSP_SAMPLES_PER_BLOCK);
    arm_fir_init_q31(&FirRight, NUM_TAPS, (q31_t *)&firCoeffQ31[0], &firRightStateq31[0], DSP_SAMPLES_PER_BLOCK);
    //convert FIR filter coefficients to Q31
    arm_float_to_q31(&firCoeffF32[0], &firCoeffQ31[0],NUM_TAPS);

    //init Real FFT instance
    arm_rfft_init_q31(&arm_rfft_sR_q31_len128Left,DSP_SAMPLES_PER_BLOCK,0,1);
    arm_rfft_init_q31(&arm_rfft_sR_q31_len128Right,DSP_SAMPLES_PER_BLOCK,0,1);

    OSTaskCreate(&dspTaskTCB,
                "DSP Task ",
                dspTask,
                (void *) 0,
                APP_CFG_DSP_TASK_PRIO,
                &dspTaskStk[0],
                (APP_CFG_DSP_TASK_STK_SIZE / 10u),
                APP_CFG_DSP_TASK_STK_SIZE,
                0,
                0,
                (void *) 0,
                (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                &os_err);

    OSSemCreate(&dspFullStop, "DMA Stopped", 0, &os_err);
    CODECInit();
    I2SInit(DSP_SSIZE_CODE_32BIT);
    DSPSampleRateSet(CODEC_SRATE_CODE_48K);
    DSPSampleSizeSet(DSP_SSIZE_CODE_32BIT);
    DMAInit(&dspInBuffer[0][0], &dspOutBuffer[0][0]);
    I2S_RX_ENABLE();
    I2S_TX_ENABLE();
}

/*******************************************************************************************
* dspTask
*******************************************************************************************/
static void dspTask(void *p_arg){
    OS_ERR os_err;
    INT8U buffer_index;
    (void)p_arg;
    while(1){
        DB0_TURN_OFF();                             /* Turn off debug bit while waiting */
        buffer_index = DMAInPend(0, &os_err);
        DB0_TURN_ON();
        // DSP code goes here.
        //AM Generator Code
        //to floating point
        arm_q31_to_float(&dspInBuffer[DSP_LEFT_CH][buffer_index].samples[0],&BufferLeft[0],DSP_SAMPLES_PER_BLOCK);
        arm_q31_to_float(&dspInBuffer[DSP_RIGHT_CH][buffer_index].samples[0],&BufferRight[0],DSP_SAMPLES_PER_BLOCK);
        //modulate 2kHz with 20kHz
        arm_mult_f32(&cosineVals[0],&BufferLeft[0],&ModBufferLeft[0],DSP_SAMPLES_PER_BLOCK);
        arm_mult_f32(&cosineVals[0],&BufferRight[0],&ModBufferRight[0],DSP_SAMPLES_PER_BLOCK);
        //demodulate 18kHz and 22kHz back to 2kHz, but we now have values at 38kHz and 42kHz which will alias down to 6kHz and 10kHz
        arm_mult_f32(&cosineVals[0],&ModBufferLeft[0],&DemodBufferLeft[0],DSP_SAMPLES_PER_BLOCK);
        arm_mult_f32(&cosineVals[0],&ModBufferRight[0],&DemodBufferRight[0],DSP_SAMPLES_PER_BLOCK);
        //to fixed point
        arm_float_to_q31(&DemodBufferLeft[0],&dspInBuffer[DSP_LEFT_CH][buffer_index].samples[0],DSP_SAMPLES_PER_BLOCK);
        arm_float_to_q31(&DemodBufferRight[0],&dspInBuffer[DSP_RIGHT_CH][buffer_index].samples[0],DSP_SAMPLES_PER_BLOCK);

        //FIR Filter: LPF, 43 Taps, cutoff @ 4kHz
        arm_fir_q31(&FirLeft,&dspInBuffer[DSP_LEFT_CH][buffer_index].samples[0],&dspOutBuffer[DSP_LEFT_CH][buffer_index].samples[0],DSP_SAMPLES_PER_BLOCK);
        arm_fir_q31(&FirRight,&dspInBuffer[DSP_RIGHT_CH][buffer_index].samples[0],&dspOutBuffer[DSP_RIGHT_CH][buffer_index].samples[0],DSP_SAMPLES_PER_BLOCK);

        if((buffer_index == 1)&&(dspStopReqFlag == 1)){
            OSSemPost(&dspFullStop,OS_OPT_POST_1,&os_err);
        }
    }
}

/*******************************************************************************************
* DSPSampleSizeSet
* To set sample size you must set word size on both the CODEC and I2S
* Note: Does not change DMA or buffer word size which can be changed independently.
*******************************************************************************************/
void DSPSampleSizeSet(INT8U size_code){
    (void)CODECSetSampleSize(size_code);
    I2SWordSizeSet(size_code);
    dspParams.ssize = dspCodeToSize[size_code];
}
/*******************************************************************************************
* DSPSampleSizeGet
* To read current sample size code
*******************************************************************************************/
INT8U DSPSampleSizeGet(void){
    return dspParams.ssize;
}
/*******************************************************************************************
* DSPSampleRateGet
* To read current sample rate code
*******************************************************************************************/
INT16U DSPSampleRateGet(void){
    return dspParams.srate;
}
/*******************************************************************************************
* DSPSampleRateSet
* To set sample rate you set the rate on the CODEC
*******************************************************************************************/
void DSPSampleRateSet(INT8U rate_code){
    (void)CODECSetSampleRate(rate_code);
    dspParams.srate = dspCodeToRate[rate_code];
}
/*******************************************************************************************
* DSPStart
* Enable DMA to fill block with samples
*******************************************************************************************/
void DSPStartReq(void){

    dspStopReqFlag = 0;
    DMAStart();
    CODECEnable();
    CODECSetPage(0x00);
    CODECDefaultConfig();
    CODECHeadphoneOutOn();

}
/*******************************************************************************************
* DSPStop
* Disable DA after input/output buffers are full
*******************************************************************************************/
void DSPStopReq(void){

    dspStopReqFlag = 1;
    DMAStopFull();

}
/****************************************************************************************
 * DSP signal when buffer is full and DMA stopped
 * 04/16/2020 TDM
 ***************************************************************************************/

void DSPStopFullPend(OS_TICK tout, OS_ERR *os_err_ptr){
    OSSemPend(&dspFullStop, tout, OS_OPT_PEND_BLOCKING,(void *)0, os_err_ptr);
}
/****************************************************************************************
 * Return a pointer to the requested buffer
 * 04/16/2020 TDM
 ***************************************************************************************/

INT32S *DSPBufferGet(BUFF_ID_T buff_id){
    INT32S *buf_ptr = (void*)0;
    if(buff_id == LEFT_IN){
        buf_ptr = (INT32S *)&dspInBuffer[DSP_LEFT_CH][0];
    }else if(buff_id == RIGHT_IN){
        buf_ptr = (INT32S *)&dspInBuffer[DSP_RIGHT_CH][0];
    }else if(buff_id == RIGHT_OUT){
        buf_ptr = (INT32S *)&dspOutBuffer[DSP_RIGHT_CH][0];
    }else if(buff_id == LEFT_OUT){
        buf_ptr = (INT32S *)&dspOutBuffer[DSP_LEFT_CH][0];
    }else{
    }
    return buf_ptr;
}


