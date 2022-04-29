/*******************************************************************************************
 * K65DMA.c
 * This version sets up the DMA for input and output based on DSP_IN_EN and DSP_OUT_EN.
 * Currently set up for ping-pong buffers on both inputs and outputs, only one channel.
 * 04/06/2017 Todd Morton
 ******************************************************************************************/

/*******************************************************************************************
* Include files
*******************************************************************************************/
#include "MCUType.h"
#include "app_cfg.h"
#include "os.h"
#include "AppDSP.h"
#include "K65DMA.h"
#include "K65TWR_GPIO.h"
/*******************************************************************************************
* Module Defines
*******************************************************************************************/
#define DMA_IN_CH            2
#define DMA_OUT_CH           0
#define DMA_BYTES_PER_BLOCK      (DSP_SAMPLES_PER_BLOCK*DSP_BUFFER_BYTES_PER_SAMPLE)
#define DMA_BYTES_PER_BUFFER     (DSP_NUM_BLOCKS*DSP_NUM_IN_CHANNELS*DMA_BYTES_PER_BLOCK)
#define DMA_IN_CHANNEL_OFFSET    (DSP_NUM_BLOCKS*DMA_BYTES_PER_BLOCK)
#define DMA_OUT_CHANNEL_OFFSET    (DSP_NUM_BLOCKS*DMA_BYTES_PER_BLOCK)

typedef struct{
    INT8U index;
    OS_SEM flag;
}DMA_BLOCK_RDY;
/*******************************************************************************************
* Private Functions Declarations
*******************************************************************************************/
DMA_BLOCK_RDY dmaInBlockRdy;
DMA_BLOCK_RDY dmaOutBlockRdy;
/*******************************************************************************************
* Global Variables
*******************************************************************************************/
/*******************************************************************************************
* Function Code
********************************************************************************************
DMAInInit
    Initializes DMA for an input stream from ADC0 to ping-pong buffers
    Parameters: none
    Return: none
*******************************************************************************************/
void DMAInit(DSP_BLOCK_T *dsp_in_buf, DSP_BLOCK_T *dsp_out_buf){
    OS_ERR os_err;

    OSSemCreate(&dmaInBlockRdy.flag, "Block Ready", 0, &os_err);

    // dmaInBlockRdy.index indicates the buffer currently not being used by the DMA in the Ping-Pong scheme.
    // It uses the DONE bit in the CSR to determine where the DMA is at. If DONE is 1, the DMA just finished
    // the [1] block and will start filling the [0] block. So, when DONE is one, we want the processing to use
    // the [1] block to avoid collisions with the DMA.
    // Since the DMA starts with the [0] block, initialize the index to the [1] block.

    dmaInBlockRdy.index = 1;

    //enable DMA clocks
    SIM->SCGC6 |= (SIM_SCGC6_DMAMUX_MASK);
    SIM->SCGC7 |= SIM_SCGC7_DMA_MASK;

    //Make sure DMAMUX is disabled
    DMAMUX->CHCFG[DMA_IN_CH] |= DMAMUX_CHCFG_ENBL(0)|DMAMUX_CHCFG_TRIG(0);

    //Minor Loop Mapping Enabled, Round Robin Arbitration, Debug enabled
    DMA0->CR = DMA_CR_EMLM(1) | DMA_CR_ERCA(1) | DMA_CR_ERGA(1) | DMA_CR_EDBG(1);

    /**** START: DMA config for Input DMA  */

    //source address is I2S receive data register
    DMA0->TCD[DMA_IN_CH].SADDR = DMA_SADDR_SADDR(&I2S0->RDR[0]);

    //No offset for source data address.  Always read RDR
    DMA0->TCD[DMA_IN_CH].SOFF = DMA_SOFF_SOFF(0);

    //Source and destination data size
    DMA0->TCD[DMA_IN_CH].ATTR = DMA_ATTR_SMOD(0) | DMA_ATTR_SSIZE(2) | DMA_ATTR_DMOD(0) | DMA_ATTR_DSIZE(2);

    //Destination Minor Loop Offset is enabled.  After each minor loop, the destination
    //pointer jumps back to the next sample in the first channel buffer
    // NBYTES = channels*bytes per sample.
    DMA0->TCD[DMA_IN_CH].NBYTES_MLOFFYES= DMA_NBYTES_MLOFFYES_DMLOE(1) | DMA_NBYTES_MLOFFYES_SMLOE(0)
                                        | DMA_NBYTES_MLOFFYES_MLOFF(-(DMA_BYTES_PER_BUFFER)+DSP_BUFFER_BYTES_PER_SAMPLE)
                                        | DMA_NBYTES_MLOFFYES_NBYTES(DSP_NUM_IN_CHANNELS*DSP_BUFFER_BYTES_PER_SAMPLE);

    //No adjustment to source address at end of major loop.
    DMA0->TCD[DMA_IN_CH].SLAST = DMA_SLAST_SLAST(0);

    //destination buffer address
    DMA0->TCD[DMA_IN_CH].DADDR = DMA_DADDR_DADDR(dsp_in_buf);

    DMA0->TCD[DMA_IN_CH].DOFF = DMA_DOFF_DOFF(DMA_IN_CHANNEL_OFFSET);

    //Set minor loop iteration counters to number of minor loops in the major loop
    DMA0->TCD[DMA_IN_CH].CITER_ELINKNO = DMA_CITER_ELINKNO_ELINK(0)|DMA_CITER_ELINKNO_CITER(DSP_NUM_BLOCKS*DSP_SAMPLES_PER_BLOCK);
    DMA0->TCD[DMA_IN_CH].BITER_ELINKNO = DMA_BITER_ELINKNO_ELINK(0)|DMA_BITER_ELINKNO_BITER(DSP_NUM_BLOCKS*DSP_SAMPLES_PER_BLOCK);

    //After Major loop, jump back to the beginning of each channel buffer
    DMA0->TCD[DMA_IN_CH].DLAST_SGA = DMA_DLAST_SGA_DLASTSGA(-(DMA_IN_CHANNEL_OFFSET+DMA_BYTES_PER_BUFFER-DSP_BUFFER_BYTES_PER_SAMPLE));

	//Enable interrupt at half filled Rx buffer and end of major loop.
	//This allows "ping-pong" buffer processing.
	DMA0->TCD[DMA_IN_CH].CSR = DMA_CSR_BWC(3) | DMA_CSR_INTHALF(1) | DMA_CSR_INTMAJOR(1);


    /**** START: DMA config for DMA Out  */

    DMAMUX->CHCFG[DMA_OUT_CH] |= DMAMUX_CHCFG_ENBL(0)|DMAMUX_CHCFG_TRIG(0);

    //source address
    DMA0->TCD[DMA_OUT_CH].SADDR = DMA_SADDR_SADDR(dsp_out_buf);

    //No offset for single channel
    DMA0->TCD[DMA_OUT_CH].SOFF = DMA_SOFF_SOFF(DMA_OUT_CHANNEL_OFFSET);

    //Source data size
    DMA0->TCD[DMA_OUT_CH].ATTR = DMA_ATTR_SMOD(0) | DMA_ATTR_SSIZE(2) | DMA_ATTR_DMOD(0) | DMA_ATTR_DSIZE(2);

    //Destination Minor Loop Offset is enabled.  After each minor loop, the destination
    //pointer jumps back to the next sample in the first channel buffer
    // NBYTES = channels*bytes per sample.
    DMA0->TCD[DMA_OUT_CH].NBYTES_MLOFFYES= DMA_NBYTES_MLOFFYES_DMLOE(0) | DMA_NBYTES_MLOFFYES_SMLOE(1)
                                        | DMA_NBYTES_MLOFFYES_MLOFF(-(DMA_BYTES_PER_BUFFER)+DSP_BUFFER_BYTES_PER_SAMPLE)
                                        | DMA_NBYTES_MLOFFYES_NBYTES(DSP_NUM_IN_CHANNELS*DSP_BUFFER_BYTES_PER_SAMPLE);

    //No adjustment to destination address at end of major loop.
    DMA0->TCD[DMA_OUT_CH].DLAST_SGA = DMA_DLAST_SGA_DLASTSGA(0);

    DMA0->TCD[DMA_OUT_CH].DOFF = DMA_DOFF_DOFF(0);

    //Source buffer address
    DMA0->TCD[DMA_OUT_CH].DADDR = DMA_DADDR_DADDR(&I2S0->TDR[0]);

    //Set minor loop iteration counters to number of minor loops in the major loop
    DMA0->TCD[DMA_OUT_CH].CITER_ELINKNO = DMA_CITER_ELINKNO_ELINK(0)|DMA_CITER_ELINKNO_CITER(DSP_NUM_BLOCKS*DSP_SAMPLES_PER_BLOCK);
    DMA0->TCD[DMA_OUT_CH].BITER_ELINKNO = DMA_BITER_ELINKNO_ELINK(0)|DMA_BITER_ELINKNO_BITER(DSP_NUM_BLOCKS*DSP_SAMPLES_PER_BLOCK);

    //After Major loop, jump back to the beginning of each channel buffer
    DMA0->TCD[DMA_OUT_CH].SLAST = DMA_SLAST_SLAST(-(DMA_IN_CHANNEL_OFFSET+DMA_BYTES_PER_BUFFER-DSP_BUFFER_BYTES_PER_SAMPLE));

    //No output channel interrupts
    DMA0->TCD[DMA_OUT_CH].CSR = DMA_CSR_BWC(3);


    //Output channel mux I2S0-TX (13)
    DMAMUX->CHCFG[DMA_OUT_CH] = DMAMUX_CHCFG_ENBL(1)|DMAMUX_CHCFG_SOURCE(13);
    //trigger source is I2S0-RX (12)
    DMAMUX->CHCFG[DMA_IN_CH] = DMAMUX_CHCFG_ENBL(1)|DMAMUX_CHCFG_SOURCE(12);

    //enable DMA Rx interrupt
    NVIC_EnableIRQ(DMA_IN_CH);

    //All set to go, enable DMA channel(s)!
    DMA0->SERQ = DMA_SERQ_SERQ(DMA_IN_CH);


    //All set to go, enable DMA channel(s)!
    DMA0->SERQ = DMA_SERQ_SERQ(DMA_OUT_CH);

}

/****************************************************************************************
 * DMA Interrupt Handler for the sample stream
 * 08/30/2015 TDM
 ***************************************************************************************/
void DMA2_DMA18_IRQHandler(void){
    OS_ERR os_err;
    OSIntEnter();
    DB1_TURN_ON();
    DMA0->CINT = DMA_CINT_CINT(2);
    if((DMA0->TCD[DMA_IN_CH].CSR & DMA_CSR_DONE_MASK) != 0){
        dmaInBlockRdy.index = 1;      //set buffer index to opposite of DMA
    }else{
        dmaInBlockRdy.index = 0;
    }
    OSSemPost(&(dmaInBlockRdy.flag),OS_OPT_POST_1,&os_err);
    DB1_TURN_OFF();
    OSIntExit();
}
/****************************************************************************************
 * DMA signal when full or half full
 * 08/30/2015 TDM
 ***************************************************************************************/
INT8U DMAInPend(OS_TICK tout, OS_ERR *os_err_ptr){

    OSSemPend(&(dmaInBlockRdy.flag), tout, OS_OPT_PEND_BLOCKING,(void *)0, os_err_ptr);
    return dmaInBlockRdy.index;
}
/****************************************************************************************
 * DMA stop at end of major block
 * By using this, the DMA will stop filling the buffer when the DMA finishes with the
 * last sample in the block.
 * 04/16/2020 TDM
 ***************************************************************************************/
void DMAStopFull(void){

    DMA0->TCD[DMA_IN_CH].CSR |= DMA_CSR_DREQ_MASK;

}
/****************************************************************************************
 * DMA start.
 * It has to clear the I2S FIFO overflow flag because the I2S keeps sending samples when
 * the DMA is stopped. It would be nice to do this in a better way, like stop the I2S.
 * 04/16/2020 TDM
 ***************************************************************************************/
void DMAStart(void){

    DMA0->TCD[DMA_IN_CH].CSR &= ~DMA_CSR_DREQ_MASK;
    I2S0->RCSR |= I2S_RCSR_SEF_MASK|I2S_RCSR_FEF_MASK;

    //All set to go, enable DMA channel(s)!
    DMA0->SERQ = DMA_SERQ_SERQ(DMA_IN_CH);
    DMA0->SERQ = DMA_SERQ_SERQ(DMA_OUT_CH);

}

