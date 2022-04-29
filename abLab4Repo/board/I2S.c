/*****************************************************************************************************
* I2S.c
*
* Initializes the I2S transmit and receive. In this case I2S0, the only one available for the K65 and
* the correct pin mapping for Aaron's CODEC board.
* Generates a 12MHz MCLK for the CODEC. The CODEC then generates the BCLK and WCLK's.
* Based mostly on Aaron's code.
* To change sample size, both the CODEC and the I2S module must be set to the correct size.
* 09/03/2015 Todd Morton
*****************************************************************************************************/
/*****************************************************************************************************
* Include files
*****************************************************************************************************/
#include "MCUType.h"
#include "I2S.h"
/*****************************************************************************************************
* void I2SInit(INT32U u32TxFIFOWm, INT32U u32RxFIFOWm)
* Based mostly on Aaron's code.
*  PARAMETERS: sizecode:
*   0x00: 16-bit samples
*   0x01: 20-bit samples
*   0x10: 24-bit samples
*   0x11: 32-bit samples
*
* 09/03/2015 Todd Morton
* 04/03/2019 Todd Morton Added sample size parameter
*****************************************************************************************************/
void I2SInit(INT8U size_code){

    INT8U i2s_word_size;

    switch(size_code){
    case 0x0:
        i2s_word_size = 15; //for 16-bit words
        break;
    case 0x1:
        i2s_word_size = 19; //for 20-bit words
        break;
    case 0x2:
        i2s_word_size = 23; //for 24-bit words
        break;
    case 0x3:
        i2s_word_size = 31; //for 32-bit words
        break;
    default:
        i2s_word_size = 31;
        break;
    }

    SIM->SCGC6 |= SIM_SCGC6_I2S_MASK;   //enable I2S clock gate

    //IO Init
    //clear all PCR bits, except MUX
    PORTE->PCR[6] &= PORT_PCR_MUX_MASK;
    PORTE->PCR[12] &= PORT_PCR_MUX_MASK;
    PORTE->PCR[11] &= PORT_PCR_MUX_MASK;
    PORTE->PCR[7] &= PORT_PCR_MUX_MASK;
    PORTE->PCR[10] &= PORT_PCR_MUX_MASK;

    //MUX I2S pins
    PORTE->PCR[6] |= PORT_PCR_MUX(0x04);   //MCLK output
    PORTE->PCR[12] |= PORT_PCR_MUX(0x04);  //BCLK output
    PORTE->PCR[11] |= PORT_PCR_MUX(0x04);  //FCLK output
    PORTE->PCR[7] |= PORT_PCR_MUX(0x04);   //RX Data
    PORTE->PCR[10] |= PORT_PCR_MUX(0x04);  //TX Data

    // Provide a MCLK output for the CODEC. In this case set to fMCLK = 12MHz.
    // This allows the CODEC to generate BCLK to 48ksps or 44.1ksps with no error.
    // output = input * [(I2SFRAC+1) / (I2SDIV+1) ] -> 12.00M = (180M* (65/975))
    I2S0->MDR |= I2S_MDR_FRACT(64) | I2S_MDR_DIVIDE(974);
    //MCLK Divider uses system clock as input and MCLK output is enabled.
    I2S0->MCR |= I2S_MCR_MOE_MASK | I2S_MCR_MICS(0);

	/****************************************************************************
	 * I2S Transmitter Initialization
	 ****************************************************************************/
	I2S0->TCR1 |= I2S_TCR1_TFW(FIFO_TX_WM);

    // For using TX BLCK and FS CLK in synch mode, TX must be Asynch and Rx synch. BCLK generated externally
    I2S0->TCR2 |= I2S_TCR2_SYNC(0)  |        // master mode(Async mode)
                 I2S_TCR2_MSEL(1)  |        // MSEL = MCLK
                 I2S_TCR2_BCP_MASK;         // BCP = drive on falling edge, sample on rising edge

	/*transmit data channel 0 enabled */
	I2S0->TCR3 |= I2S_TCR3_WDFL(0)|I2S_TCR3_TCE(1);
//
	I2S0->TCR4 |= I2S_TCR4_FRSZ(1)  |     // 2 (1+1) words in a frame
	            I2S_TCR4_SYWD(i2s_word_size) |     // bits in a word
	            I2S_TCR4_MF_MASK  |     // MSB First
	            I2S_TCR4_FSE_MASK |     // one bit early
	            I2S_TCR4_FSP_MASK;      // frame active low

//	/*24-bit first and following words */
	I2S0->TCR5 |= I2S_TCR5_WNW(i2s_word_size) |      // word N width, 32 bits
	            I2S_TCR5_W0W(i2s_word_size) |      // word 0 width, 32 bits
	            I2S_TCR5_FBT(31);       // left justify for q31 data type

	/* enable DMA transfer on FIFO WM */
	I2S0->TCSR |= I2S_TCSR_FRDE_MASK|I2S_TCSR_FR_MASK;

    /****************************************************************************
     * I2S Receiver Initialization
     ****************************************************************************/
	I2S0->RCR1 |= I2S_RCR1_RFW(FIFO_RX_WM);
	I2S0->RCR2 |= I2S_RCR2_SYNC(1);   //Rx synch with Tx
	I2S0->RCR3 |= I2S_RCR3_WDFL(0)|I2S_RCR3_RCE(1);

    I2S0->RCR4 |= I2S_RCR4_FRSZ(1)  |     // 2 words in a frame
                I2S_RCR4_SYWD(i2s_word_size) |     // 32 bits in a word
                I2S_RCR4_MF_MASK  |     // MSB
                I2S_RCR4_FSE_MASK |     // one bit early
                I2S_RCR4_FSP_MASK;      // frame active low

    I2S0->RCR5 |= I2S_RCR5_WNW(i2s_word_size) |      // word N width, 32 bits
                I2S_RCR5_W0W(i2s_word_size) |      // word 0 width, 32 bits
                I2S_RCR5_FBT(31);       // left justify for q31 data type

	I2S0->RCSR |= I2S_RCSR_FRDE_MASK|I2S_RCSR_FR_MASK;
}

void I2SWordSizeSet(INT8U size_code){
    INT8U i2s_word_size;

    switch(size_code){
    case 0x0:
        i2s_word_size = 15; //for 16-bit words
        break;
    case 0x1:
        i2s_word_size = 19; //for 20-bit words
        break;
    case 0x2:
        i2s_word_size = 23; //for 24-bit words
        break;
    case 0x3:
        i2s_word_size = 31; //for 32-bit words
        break;
    default:
        i2s_word_size = 31;
        break;
    }

    I2S_TX_DISABLE();
    I2S_RX_DISABLE();

    I2S0->TCR4 |= I2S_TCR4_SYWD(i2s_word_size);

    I2S0->TCR5 |= I2S_TCR5_WNW(i2s_word_size) |        // word N width, 32 bits
                  I2S_TCR5_W0W(i2s_word_size);         // word 0 width, 32 bits

    I2S0->RCR4 |= I2S_RCR4_SYWD(i2s_word_size);

    I2S0->RCR5 |= I2S_RCR5_WNW(i2s_word_size) |        // word N width, 32 bits
                  I2S_RCR5_W0W(i2s_word_size);         // word 0 width, 32 bits

    I2S_TX_ENABLE();
    I2S_RX_ENABLE();

}
