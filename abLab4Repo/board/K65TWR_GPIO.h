/*******************************************************************************
* K65TWR_GPIO.h - K65TWR GPIO support package
* Todd Morton, 10/08/2015
* Todd Morton, 11/25/2015 Modified for new Debug bits. See EE344, Lab5, 2015
* Todd Morton, 10/09/2018 Modified for new macros in MCUXpresso 10
* Todd Morton, 10/09/2020 Added IRQC macros
 ******************************************************************************/

#ifndef GPIO_H_
#define GPIO_H_

/*******************************************************************************
* GpioSw3Init - Initialization for SW3 on the TWR-K65 board
*               SW3 is wired active-low, there is NO pull-up on the board so an
*               internal pull-up is required.
* IRQC Parameters:
* PORT_IRQ_OFF      Interrupts disabled
* PORT_IRQ_DMA_RE   DMA on rising-edge
* PORT_IRQ_DMA_FE   DMA on falling-edge
* PORT_IRQ_DMA_EE   DMA on either edge
* PORT_IRQ_ZERO     Interrupt when 0
* PORT_IRQ_RE       Interrupt on rising-edge
* PORT_IRQ_FE       Interrupt on falling-edge
* PORT_IRQ_EE       Interrupts on either edge
* PORT_IRQ_ONE      Interrupt when 1
 ******************************************************************************/
void GpioSw3Init(INT8U irqc);

/*******************************************************************************
* GpioSw2Init - Initialization for SW2 on the TWR-K65 board
*               SW2 is wired active-low, there is a pull-up on the board so an
*               internal pull-up is not required.
* IRQC Parameters:
* PORT_IRQ_OFF      Interrupts disabled
* PORT_IRQ_DMA_RE   DMA on rising-edge
* PORT_IRQ_DMA_FE   DMA on falling-edge
* PORT_IRQ_DMA_EE   DMA on either edge
* PORT_IRQ_ZERO     Interrupt when 0
* PORT_IRQ_RE       Interrupt on rising-edge
* PORT_IRQ_FE       Interrupt on falling-edge
* PORT_IRQ_EE       Interrupts on either edge
* PORT_IRQ_ONE      Interrupt when 1
 ******************************************************************************/
void GpioSw2Init(INT8U irqc);

/*******************************************************************************
* GpioLED8Init - Initialization for LED8 port to an output, LED off.
* 10/16/2017, TDM
 ******************************************************************************/
void GpioLED8Init(void);

/*******************************************************************************
* GpioLED9Init - Initialization for LED9 port to an output, LED off.
* 10/16/2017, TDM
 ******************************************************************************/
void GpioLED9Init(void);

/*******************************************************************************
* GpioDBugBitsInit - Initialization for Debug bits, each bit is cleared to 0.
* 10/16/2017, TDM
 ******************************************************************************/
void GpioDBugBitsInit(void);

/*******************************************************************************
 * Port interrupt control macros - value passed to IRQC bits
 ******************************************************************************/
#define PORT_IRQ_OFF       0    //Interrupts disabled
#define PORT_IRQ_DMA_RE    1    //DMA on rising-edge
#define PORT_IRQ_DMA_FE    2    //DMA on falling-edge
#define PORT_IRQ_DMA_EE    3    //DMA on either edge
#define PORT_IRQ_ZERO      8    //Interrupt when 0
#define PORT_IRQ_RE        9    //Interrupt on rising-edge
#define PORT_IRQ_FE       10    //Interrupt on falling-edge
#define PORT_IRQ_EE       11    //Interrupts on either edge
#define PORT_IRQ_ONE      12    //Interrupt when 1

/*******************************************************************************
 * Pin macro
 ******************************************************************************/
#define GPIO_PIN(x) (((1)<<(x & 0x1FU)))
/*******************************************************************************
 * Switch defines for SW2 (PTA4), SW3 (PTA10), LED8 (PTA28), and LED9 (PTA29)
 ******************************************************************************/
#define LED8_BIT 28U
#define LED9_BIT 29U

#define LED8_TURN_OFF()     (GPIOA->PSOR = GPIO_PIN(LED8_BIT))
#define LED8_TURN_ON()      (GPIOA->PCOR = GPIO_PIN(LED8_BIT))
#define LED8_TOGGLE()       (GPIOA->PTOR = GPIO_PIN(LED8_BIT))

#define LED9_TURN_OFF()     (GPIOA->PSOR = GPIO_PIN(LED9_BIT))
#define LED9_TURN_ON()      (GPIOA->PCOR = GPIO_PIN(LED9_BIT))
#define LED9_TOGGLE()       (GPIOA->PTOR = GPIO_PIN(LED9_BIT))

#define SW2_BIT 4U
#define SW3_BIT 10U
#define SW2_INPUT           (GPIOA->PDIR & GPIO_PIN(SW2_BIT))
#define SW3_INPUT           (GPIOA->PDIR & GPIO_PIN(SW3_BIT))

#define SW2_ISF             (PORTA->ISFR & GPIO_PIN(SW2_BIT))

/*******************************************************************************
 * Initialize SW2 interrupt. irqc is one of the IRQC macros defined above
 * Also enables a pull-up since SW2 does not have an external pull-up resistor
 ******************************************************************************/
#define SW2_INIT_IRQ(irqc)  (PORTA->PCR26=PORT_PCR_MUX(1)|PORT_PCR_PE_MASK|\
                             PORT_PCR_PS_MASK|PORT_PCR_IRQC(irqc))

#define SW2_CLR_ISF()       (PORTA->ISFR = GPIO_PIN(SW2_BIT))

/*******************************************************************************
 * #defines for debug bits
 ******************************************************************************/
#define DB0_BIT 15
#define DB1_BIT 14
#define DB2_BIT 13
#define DB3_BIT 12
#define DB4_BIT 23
#define DB5_BIT 22
#define DB6_BIT 21
#define DB7_BIT 20

#define DB0_TURN_ON() (GPIOC->PSOR = GPIO_PIN(DB0_BIT))
#define DB1_TURN_ON() (GPIOC->PSOR = GPIO_PIN(DB1_BIT))
#define DB2_TURN_ON() (GPIOC->PSOR = GPIO_PIN(DB2_BIT))
#define DB3_TURN_ON() (GPIOC->PSOR = GPIO_PIN(DB3_BIT))
#define DB4_TURN_ON() (GPIOB->PSOR = GPIO_PIN(DB4_BIT))
#define DB5_TURN_ON() (GPIOB->PSOR = GPIO_PIN(DB5_BIT))
#define DB6_TURN_ON() (GPIOB->PSOR = GPIO_PIN(DB6_BIT))
#define DB7_TURN_ON() (GPIOB->PSOR = GPIO_PIN(DB7_BIT))

#define DB0_TURN_OFF() (GPIOC->PCOR = GPIO_PIN(DB0_BIT))
#define DB1_TURN_OFF() (GPIOC->PCOR = GPIO_PIN(DB1_BIT))
#define DB2_TURN_OFF() (GPIOC->PCOR = GPIO_PIN(DB2_BIT))
#define DB3_TURN_OFF() (GPIOC->PCOR = GPIO_PIN(DB3_BIT))
#define DB4_TURN_OFF() (GPIOB->PCOR = GPIO_PIN(DB4_BIT))
#define DB5_TURN_OFF() (GPIOB->PCOR = GPIO_PIN(DB5_BIT))
#define DB6_TURN_OFF() (GPIOB->PCOR = GPIO_PIN(DB6_BIT))
#define DB7_TURN_OFF() (GPIOB->PCOR = GPIO_PIN(DB7_BIT))

#define DB0_TOGGLE() (GPIOC->PTOR = GPIO_PIN(DB0_BIT))
#define DB1_TOGGLE() (GPIOC->PTOR = GPIO_PIN(DB1_BIT))
#define DB2_TOGGLE() (GPIOC->PTOR = GPIO_PIN(DB2_BIT))
#define DB3_TOGGLE() (GPIOC->PTOR = GPIO_PIN(DB3_BIT))
#define DB4_TOGGLE() (GPIOB->PTOR = GPIO_PIN(DB4_BIT))
#define DB5_TOGGLE() (GPIOB->PTOR = GPIO_PIN(DB5_BIT))
#define DB6_TOGGLE() (GPIOB->PTOR = GPIO_PIN(DB6_BIT))
#define DB7_TOGGLE() (GPIOB->PTOR = GPIO_PIN(DB7_BIT))
#endif /* DBUGBITS_H_ */
