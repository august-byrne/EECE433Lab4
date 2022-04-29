/****************************************************************************************
 * K65TWR_ClkCfg
 *
 * This a standalone clock configuration module based on the old KDS clock configuration
 * system_MK65F18.c and .h. Using this allows you to not have to link in a bunch of SDK
 * modules that are not normally needed.
 * Provides a system configuration function and a global variable that contains
 * the system frequency. It configures the device and initializes the oscillator
 * (PLL) that is part of the microcontroller device.
 *
 * 09/06/2018 Todd Morton
 *
 ***************************************************************************************/

//#include <stdint.h>
#include "MCUType.h"
#include "K65TWR_ClkCfg.h"

/****************************************************************************************
 * Configure and start the system clocks based on the settings in K65TWR_ClkCfg.h
 * Todd Morton, 09/06/2018
 * *************************************************************************************/

void K65TWR_BootClock(void){

#ifdef CLOCK_SETUP
	/* Wake-up from VLLSx? */
	if((RCM->SRS0 & RCM_SRS0_WAKEUP_MASK) != 0x00U){
		/* VLLSx recovery */
		if((PMC->REGSC & PMC_REGSC_ACKISO_MASK) != 0x00U){
			/* Release hold with ACKISO: Only has an effect if recovering from VLLSx.*/
			PMC->REGSC |= PMC_REGSC_ACKISO_MASK;
    }
	}else{
		/* RTC initialization */
#ifdef SYSTEM_RTC_CR_VALUE
		SIM->SCGC6 |= SIM_SCGC6_RTC_MASK;
		if ((RTC->CR & RTC_CR_OSCE_MASK) == 0x00U) { /* Only if the OSC not enabled */
			RTC->CR = (INT32U)((RTC->CR & (INT32U)~(INT32U)
					  (RTC_CR_SC2P_MASK | RTC_CR_SC4P_MASK | RTC_CR_SC8P_MASK |
					   RTC_CR_SC16P_MASK)) | (INT32U)SYSTEM_RTC_CR_VALUE);
			RTC->CR |= (INT32U)RTC_CR_OSCE_MASK;
			RTC->CR &= (INT32U)~(INT32U)RTC_CR_CLKO_MASK;
		}
#endif
	}

  /* Power mode protection initialization */
#ifdef SYSTEM_SMC_PMPROT_VALUE
	SMC->PMPROT = SYSTEM_SMC_PMPROT_VALUE;
#endif

  /* High speed run mode enable */
#if (((SYSTEM_SMC_PMCTRL_VALUE) & SMC_PMCTRL_RUNM_MASK) == (0x03U << SMC_PMCTRL_RUNM_SHIFT))
	SMC->PMCTRL = (INT8U)((SYSTEM_SMC_PMCTRL_VALUE) & (SMC_PMCTRL_RUNM_MASK)); /* Enable HSRUN mode */
	while(SMC->PMSTAT != 0x80U) {}  /* Wait until the system is in HSRUN mode */
#endif

	/* System clock initialization */
	/* Internal reference clock trim initialization */
#if defined(SLOW_TRIM_ADDRESS)
	if ( *((IINT8U*)SLOW_TRIM_ADDRESS) != 0xFFU) {                              /* Skip if non-volatile flash memory is erased */
	    MCG->C3 = *((INT8U*)SLOW_TRIM_ADDRESS);
#endif /* defined(SLOW_TRIM_ADDRESS) */
#if defined(SLOW_FINE_TRIM_ADDRESS)
    MCG->C4 = (MCG->C4 & ~(MCG_C4_SCFTRIM_MASK)) | ((*((INT8U*) SLOW_FINE_TRIM_ADDRESS)) & MCG_C4_SCFTRIM_MASK);
#endif
#if defined(FAST_TRIM_ADDRESS)
    MCG->C4 = (MCG->C4 & ~(MCG_C4_FCTRIM_MASK)) |((*((INT8U*) FAST_TRIM_ADDRESS)) & MCG_C4_FCTRIM_MASK);
#endif
#if defined(FAST_FINE_TRIM_ADDRESS)
    MCG->C2 = (MCG->C2 & ~(MCG_C2_FCFTRIM_MASK)) | ((*((INT8U*)FAST_TRIM_ADDRESS)) & MCG_C2_FCFTRIM_MASK);
#endif /* defined(FAST_FINE_TRIM_ADDRESS) */
#if defined(SLOW_TRIM_ADDRESS)
  }
#endif /* defined(SLOW_TRIM_ADDRESS) */

	/* Set system prescalers and clock sources */
	SIM->CLKDIV1 = SYSTEM_SIM_CLKDIV1_VALUE; /* Set system prescalers */
	SIM->SOPT1 = ((SIM->SOPT1) & (INT32U)(~(SIM_SOPT1_OSC32KSEL_MASK))) | ((SYSTEM_SIM_SOPT1_VALUE) & (SIM_SOPT1_OSC32KSEL_MASK)); /* Set 32 kHz clock source (ERCLK32K) */
	SIM->SOPT2 = ((SIM->SOPT2) & (INT32U)(~(SIM_SOPT2_PLLFLLSEL_MASK))) | ((SYSTEM_SIM_SOPT2_VALUE) & (SIM_SOPT2_PLLFLLSEL_MASK)); /* Selects the high frequency clock for various peripheral clocking options. */
	SIM->SOPT2 = ((SIM->SOPT2) & (INT32U)(~(SIM_SOPT2_TPMSRC_MASK))) | ((SYSTEM_SIM_SOPT2_VALUE) & (SIM_SOPT2_TPMSRC_MASK)); /* Selects the clock source for the TPM counter clock. */
#if ((MCG_MODE == MCG_MODE_FEI) || (MCG_MODE == MCG_MODE_FBI) || (MCG_MODE == MCG_MODE_BLPI))
	/* Set MCG and OSC */
#if  ((((SYSTEM_OSC_CR_VALUE) & OSC_CR_ERCLKEN_MASK) != 0x00U) || ((((SYSTEM_MCG_C5_VALUE) & MCG_C5_PLLCLKEN_MASK) != 0x00U) && (((SYSTEM_MCG_C7_VALUE) & MCG_C7_OSCSEL_MASK) == 0x00U)))
	/* SIM_SCGC5: PORTA=1 */
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
	/* PORTA_PCR18: ISF=0,MUX=0 */
	PORTA->PCR[18] &= (INT32U)~(INT32U)((PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07)));
	if (((SYSTEM_MCG_C2_VALUE) & MCG_C2_EREFS_MASK) != 0x00U) {
	    /* PORTA_PCR19: ISF=0,MUX=0 */
	    PORTA->PCR[19] &= (INT32U)~(INT32U)((PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07)));
	}
#endif
	MCG->SC = SYSTEM_MCG_SC_VALUE;       /* Set SC (fast clock internal reference divider) */
	MCG->C1 = SYSTEM_MCG_C1_VALUE;       /* Set C1 (clock source selection, FLL ext. reference divider, int. reference enable etc.) */
	/* Check that the source of the FLL reference clock is the requested one. */
	if (((SYSTEM_MCG_C1_VALUE) & MCG_C1_IREFS_MASK) != 0x00U) {
	    while((MCG->S & MCG_S_IREFST_MASK) == 0x00U) {}
	}else{
	    while((MCG->S & MCG_S_IREFST_MASK) != 0x00U) {}
	}
	MCG->C2 = (MCG->C2 & (INT8U)(~(MCG_C2_FCFTRIM_MASK))) | (SYSTEM_MCG_C2_VALUE & (INT8U)(~(MCG_C2_LP_MASK))); /* Set C2 (freq. range, ext. and int. reference selection etc. excluding trim bits; low power bit is set later) */
	MCG->C4 = ((SYSTEM_MCG_C4_VALUE) & (INT8U)(~(MCG_C4_FCTRIM_MASK | MCG_C4_SCFTRIM_MASK))) | (MCG->C4 & (MCG_C4_FCTRIM_MASK | MCG_C4_SCFTRIM_MASK)); /* Set C4 (FLL output; trim values not changed) */
	OSC->CR = SYSTEM_OSC_CR_VALUE;       /* Set OSC_CR (OSCERCLK enable, oscillator capacitor load) */
	MCG->C7 = SYSTEM_MCG_C7_VALUE;       /* Set C7 (OSC Clock Select) */

#else /* MCG_MODE */
	/* Set MCG and OSC */
#if  (((SYSTEM_OSC_CR_VALUE) & OSC_CR_ERCLKEN_MASK) != 0x00U) || (((SYSTEM_MCG_C7_VALUE) & MCG_C7_OSCSEL_MASK) == 0x00U)
	/* SIM_SCGC5: PORTA=1 */
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
	/* PORTA_PCR18: ISF=0,MUX=0 */
	PORTA->PCR[18] &= (INT32U)~(INT32U)((PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07)));
	if (((SYSTEM_MCG_C2_VALUE) & MCG_C2_EREFS_MASK) != 0x00U) {
	    /* PORTA_PCR19: ISF=0,MUX=0 */
	    PORTA->PCR[19] &= (INT32U)~(INT32U)((PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07)));
	}
#endif
	MCG->SC = SYSTEM_MCG_SC_VALUE;       /* Set SC (fast clock internal reference divider) */
	MCG->C2 = (MCG->C2 & (INT8U)(~(MCG_C2_FCFTRIM_MASK))) | (SYSTEM_MCG_C2_VALUE & (INT8U)(~(MCG_C2_LP_MASK))); /* Set C2 (freq. range, ext. and int. reference selection etc. excluding trim bits; low power bit is set later) */
	OSC->CR = SYSTEM_OSC_CR_VALUE;       /* Set OSC_CR (OSCERCLK enable, oscillator capacitor load) */
	MCG->C7 = SYSTEM_MCG_C7_VALUE;       /* Set C7 (OSC Clock Select) */
#if (MCG_MODE == MCG_MODE_PEE)
	MCG->C1 = (SYSTEM_MCG_C1_VALUE) | MCG_C1_CLKS(0x02); /* Set C1 (clock source selection, FLL ext. reference divider, int. reference enable etc.) - PBE mode*/
#else
	MCG->C1 = SYSTEM_MCG_C1_VALUE;       /* Set C1 (clock source selection, FLL ext. reference divider, int. reference enable etc.) */
#endif
	if ((((SYSTEM_MCG_C2_VALUE) & MCG_C2_EREFS_MASK) != 0x00U) && (((SYSTEM_MCG_C7_VALUE) & MCG_C7_OSCSEL_MASK) == 0x00U)) {
	    while((MCG->S & MCG_S_OSCINIT0_MASK) == 0x00U) { /* Check that the oscillator is running */
	    }
	}
	/* Check that the source of the FLL reference clock is the requested one. */
	if (((SYSTEM_MCG_C1_VALUE) & MCG_C1_IREFS_MASK) != 0x00U) {
	    while((MCG->S & MCG_S_IREFST_MASK) == 0x00U) {}
	}else{
	    while((MCG->S & MCG_S_IREFST_MASK) != 0x00U) {}
	}
	MCG->C4 = ((SYSTEM_MCG_C4_VALUE)  & (INT8U)(~(MCG_C4_FCTRIM_MASK | MCG_C4_SCFTRIM_MASK))) | (MCG->C4 & (MCG_C4_FCTRIM_MASK | MCG_C4_SCFTRIM_MASK)); /* Set C4 (FLL output; trim values not changed) */
#endif /* MCG_MODE */

  /* USB UTMI PHY PLL initialization */
#ifdef SYSTEM_USBPHY_ANACTRL_VALUE
	SIM->SOPT2 |= SIM_SOPT2_USBREGEN_MASK; /* Enable USB PHY PLL regulator */
	SIM->SCGC3 |= SIM_SCGC3_USBHSPHY_MASK; /* Enable USB PHY clock gate */
	/* Use LPTMR to wait for 1ms */
	SIM->SCGC5 |= SIM_SCGC5_LPTMR_MASK;  /* Allow software control of LPMTR */
	LPTMR0->CMR = LPTMR_CMR_COMPARE(0);  /* Default 1 LPO tick */
	LPTMR0->CSR = (LPTMR_CSR_TCF_MASK | LPTMR_CSR_TPS(0x00));
	LPTMR0->PSR = (LPTMR_PSR_PCS(0x01) | LPTMR_PSR_PBYP_MASK); /* Clock source: LPO, Prescaler bypass enable */
	LPTMR0->CSR = LPTMR_CSR_TEN_MASK;    /* LPMTR enable */
	while((LPTMR0->CSR & LPTMR_CSR_TCF_MASK) == 0u) {}
	LPTMR0->CSR = 0x00;                  /* Disable LPTMR */
	SIM->USBPHYCTL = SIM_USBPHYCTL_USB3VOUTTRG(6); /* Trim the USB regulator output to be 3.13V */
	USBPHY_TRIM_OVERRIDE_EN = USBPHY_TRIM_OVERRIDE_EN_TRIM_DIV_SEL_OVERRIDE_MASK; /* Enable override of PLL_DIV_SEL IFR value */
	USBPHY->CTRL &= ~USBPHY_CTRL_SFTRST_MASK; /* Release PHY from reset */
	USBPHY->CTRL &= ~USBPHY_CTRL_CLKGATE_MASK; /* Clear CLKGATE to run clocks */
	USBPHY->PLL_SIC |= USBPHY_PLL_SIC_PLL_POWER_MASK; /* Power up PLL */
#if (CPU_XTAL_CLK_HZ == 24000000U)   /* Setup the USB PLL feedback loop divider */
	USBPHY->PLL_SIC |= USBPHY_PLL_SIC_PLL_DIV_SEL(0);
#elif (CPU_XTAL_CLK_HZ == 16000000U)
	USBPHY->PLL_SIC |= USBPHY_PLL_SIC_PLL_DIV_SEL(1);
#elif (CPU_XTAL_CLK_HZ == 12000000U)
	USBPHY->PLL_SIC |= USBPHY_PLL_SIC_PLL_DIV_SEL(2);
#endif
	USBPHY->PLL_SIC &= ~USBPHY_PLL_SIC_PLL_BYPASS_MASK; /* Clear bypass bit */
	USBPHY->PLL_SIC |= USBPHY_PLL_SIC_PLL_EN_USB_CLKS_MASK;	 /* Enable USB clock output from USB PHY PLL */
	USBPHY->CTRL |= (USBPHY_CTRL_AUTORESUME_EN_MASK | USBPHY_CTRL_ENAUTOCLR_CLKGATE_MASK | USBPHY_CTRL_ENAUTOCLR_PHY_PWD_MASK); /* Enable auto resume feature, enable auo-clear CLKGATE bit if there is wakeup event while USB suspended, enable the feature to auto-clear the PWD register bits in USBPHY_PWD if there is wakeup event while USB is suspended.) */
	while(!(USBPHY->PLL_SIC & USBPHY_PLL_SIC_PLL_LOCK_MASK)) {}
	USBPHY->PLL_SIC |= USBPHY_PLL_SIC_PLL_ENABLE_MASK; /* Enable PLL output */
	USBPHY->ANACTRL = (USBPHY->ANACTRL & (INT32U)(~(USBPHY_ANACTRL_PFD_CLK_SEL_MASK | USBPHY_ANACTRL_PFD_FRAC_MASK)))
                  | (SYSTEM_USBPHY_ANACTRL_VALUE & (USBPHY_ANACTRL_PFD_CLK_SEL_MASK | USBPHY_ANACTRL_PFD_FRAC_MASK)); /* Setup the PFD module */
	USBPHY->ANACTRL &= ~USBPHY_ANACTRL_PFD_CLKGATE_MASK;
#endif

	/* Common for all MCG modes */

	/* PLL clock can be used to generate clock for some devices regardless of clock generator (MCGOUTCLK) mode. */
	MCG->C5 = (SYSTEM_MCG_C5_VALUE) & (INT8U)(~(MCG_C5_PLLCLKEN_MASK)); /* Set C5 (PLL settings, PLL reference divider etc.) */
	MCG->C6 = (SYSTEM_MCG_C6_VALUE) & (INT8U)~(MCG_C6_PLLS_MASK); /* Set C6 (PLL select, VCO divider etc.) */
	if ((SYSTEM_MCG_C5_VALUE) & MCG_C5_PLLCLKEN_MASK) {
	    MCG->C5 |= MCG_C5_PLLCLKEN_MASK;   /* PLL clock enable in mode other than PEE or PBE */
	}

	/* BLPI and BLPE MCG mode specific */
#if ((MCG_MODE == MCG_MODE_BLPI) || (MCG_MODE == MCG_MODE_BLPE))
	MCG->C2 |= (MCG_C2_LP_MASK);         /* Disable FLL and PLL in bypass mode */
	/* PEE and PBE MCG mode specific */
#elif ((MCG_MODE == MCG_MODE_PBE) || (MCG_MODE == MCG_MODE_PEE))
	MCG->C11 = SYSTEM_MCG_C11_VALUE;      /* Set C11 (Select PLL used to derive MCGOUT */
	MCG->C6 |= (MCG_C6_PLLS_MASK);       /* Set C6 (PLL select, VCO divider etc.) */
#if ((MCG_MODE == MCG_MODE_PEE) && (((SYSTEM_MCG_C11_VALUE) & MCG_C11_PLLCS_MASK) == 0x00U))
	while((MCG->S & MCG_S_LOCK0_MASK) == 0x00U) { /* Wait until PLL is locked*/
	}
#endif
#if (MCG_MODE == MCG_MODE_PEE)
	MCG->C1 &= (INT8U)~(MCG_C1_CLKS_MASK);
#endif
#endif

	/* Clock mode status check */
#if ((MCG_MODE == MCG_MODE_FEI) || (MCG_MODE == MCG_MODE_FEE))
	while((MCG->S & MCG_S_CLKST_MASK) != 0x00U) { /* Wait until output of the FLL is selected */
	}
	/* Use LPTMR to wait for 1ms for FLL clock stabilization */
    SIM->SCGC5 |= SIM_SCGC5_LPTMR_MASK;  /* Allow software control of LPMTR */
    LPTMR0->CMR = LPTMR_CMR_COMPARE(0);  /* Default 1 LPO tick */
    LPTMR0->CSR = (LPTMR_CSR_TCF_MASK | LPTMR_CSR_TPS(0x00));
    LPTMR0->PSR = (LPTMR_PSR_PCS(0x01) | LPTMR_PSR_PBYP_MASK); /* Clock source: LPO, Prescaler bypass enable */
    LPTMR0->CSR = LPTMR_CSR_TEN_MASK;    /* LPMTR enable */
    while((LPTMR0->CSR & LPTMR_CSR_TCF_MASK) == 0u) {
    }
    LPTMR0->CSR = 0x00;                  /* Disable LPTMR */
    SIM->SCGC5 &= (INT32U)~(INT32U)SIM_SCGC5_LPTMR_MASK;
#elif ((MCG_MODE == MCG_MODE_FBI) || (MCG_MODE == MCG_MODE_BLPI))
    while((MCG->S & MCG_S_CLKST_MASK) != 0x04U) { /* Wait until internal reference clock is selected as MCG output */
    }
#elif ((MCG_MODE == MCG_MODE_FBE) || (MCG_MODE == MCG_MODE_PBE) || (MCG_MODE == MCG_MODE_BLPE))
    while((MCG->S & MCG_S_CLKST_MASK) != 0x08U) { /* Wait until external reference clock is selected as MCG output */
    }
#elif (MCG_MODE == MCG_MODE_PEE)
    while((MCG->S & MCG_S_CLKST_MASK) != 0x0CU) { /* Wait until output of the PLL is selected */
    }
    while(MCG->S2 != SYSTEM_MCG_C11_VALUE) { /* Wait until output of the correct PLL is selected */
    }
#elif ((MCG_MODE == MCG_MODE_FBE) || (MCG_MODE == MCG_MODE_BLPE))
    while((MCG->S & MCG_S_CLKST_MASK) != 0x08U) { /* Wait until external reference clock is selected as MCG output */
    }
#endif

    /* Very-low-power run mode enable */
#if (((SYSTEM_SMC_PMCTRL_VALUE) & SMC_PMCTRL_RUNM_MASK) == (0x02U << SMC_PMCTRL_RUNM_SHIFT))
    SMC->PMCTRL = (INT8U)((SYSTEM_SMC_PMCTRL_VALUE) & (SMC_PMCTRL_RUNM_MASK)); /* Enable VLPR mode */
    while(SMC->PMSTAT != 0x04U) {        /* Wait until the system is in VLPR mode */
    }
#endif

#if defined(SYSTEM_SIM_CLKDIV2_VALUE)
    SIM->CLKDIV2 = ((SIM->CLKDIV2) & (INT32U)(~(SIM_CLKDIV2_USBFRAC_MASK | SIM_CLKDIV2_USBDIV_MASK))) | ((SYSTEM_SIM_CLKDIV2_VALUE) & (SIM_CLKDIV2_USBFRAC_MASK | SIM_CLKDIV2_USBDIV_MASK)); /* Selects the USB clock divider. */
#endif
#if defined(SYSTEM_SIM_CLKDIV3_VALUE)
    SIM->CLKDIV3 = ((SIM->CLKDIV3) & (INT32U)(~(SIM_CLKDIV3_PLLFLLFRAC_MASK | SIM_CLKDIV3_PLLFLLDIV_MASK))) | ((SYSTEM_SIM_CLKDIV3_VALUE) & (SIM_CLKDIV3_PLLFLLFRAC_MASK | SIM_CLKDIV3_PLLFLLDIV_MASK)); /* Selects the PLLFLL clock divider. */
#endif

    /* PLL loss of lock interrupt request initialization */
    if (((SYSTEM_MCG_C6_VALUE) & MCG_C6_LOLIE0_MASK) != 0U) {
        NVIC_EnableIRQ(MCG_IRQn);          /* Enable PLL loss of lock interrupt request */
    }
#endif
}
