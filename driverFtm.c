/*
 * driverFtm.c
 *
 *  Created on: Oct 16, 2019
 *      Author: mlste
 */
#include "driverFtm.h"
#include "MK64F12.h"
#include <stdint.h>
#include "gpio.h"
#include "board.h"


#define DRIVER_FTM_CLK_SOURCE 1
#define DRIVER_FTM_CLK_TIMER_AMAUNT 4
#define DRIVER_FTM_CNTIN_INIT 0x0000
#define DRIVER_FTM_CNTIN_MOD 0xFFFF
#define DRIVER_FTM_PSC DRIVER_FTM_PSC_x32

typedef enum
{
	DRIVER_FTM_PSC_x1 = 0x00,
	DRIVER_FTM_PSC_x2 = 0x01,
	DRIVER_FTM_PSC_x4 = 0x02,
	DRIVER_FTM_PSC_x8 = 0x03,
	DRIVER_FTM_PSC_x16 = 0x04,
	DRIVER_FTM_PSC_x32 = 0x05,
	DRIVER_FTM_PSC_x64 = 0x06,
	DRIVER_FTM_PSC_x128 = 0x07,
} FTM_Prescal_t;


FTM_Type ** ftmTimers = FTM_BASE_PTRS;
uint32_t * clock_gates = {&(SIM->SCGC6), &(SIM->SCGC6), &(SIM->SCGC6),  &(SIM->SCGC3)};
uint32_t * clock_masks = {SIM_SCGC6_FTM0_MASK, SIM_SCGC6_FTM1_MASK, SIM_SCGC6_FTM2_MASK, SIM_SCGC3_FTM3_MASK};
uint8_t * irqEnable=FTM_IRQS;
void driverFtmInit(int witchFtm){
	//clock gating
	gpioMode(PIN_SCK, OUTPUT);
	if(witchFtm>DRIVER_FTM_CLK_TIMER_AMAUNT){
		witchFtm=DRIVER_FTM_CLK_TIMER_AMAUNT;
	}
	SIM->SCGC6 |=SIM_SCGC6_FTM0_MASK;//clock gating del perfiferico
	//clock_gates[witchFtm] = (clock_gates[witchFtm] & ~clock_masks[witchFtm]) | clock_masks[witchFtm];
	/*
	ftmTimers[witchFtm]->MODE = (ftmTimers[witchFtm]->MODE & ~FTM_MODE_FTMEN_MASK) | FTM_MODE_FTMEN_MASK;//pongo el ftmen en 1
	ftmTimers[witchFtm]->SC = (ftmTimers[witchFtm]->SC & ~FTM_SC_CLKS_MASK) | FTM_SC_CLKS(DRIVER_FTM_CLK_SOURCE); //indico que utilizo el sitme clock 50M
	ftmTimers[witchFtm]->SC = (ftmTimers[witchFtm]->SC & ~FTM_SC_PS_MASK) | FTM_SC_PS(DRIVER_FTM_PSC_x2); //indico que utilizo el sitem clock 50M
	ftmTimers[witchFtm]->CNTIN = (ftmTimers[witchFtm]->CNTIN & ~FTM_CNTIN_INIT_MASK) | FTM_CNTIN_INIT(DRIVER_FTM_CNTIN_INIT);//seteo el valor inicial del contador
	ftmTimers[witchFtm]->MOD = (ftmTimers[witchFtm]->MOD & ~FTM_MOD_MOD_MASK)|FTM_MOD_MOD(DRIVER_FTM_CNTIN_MOD);//seteo el valor al que llega el contador

	ftmTimers[witchFtm]->SC = (ftmTimers[witchFtm]->SC & ~ FTM_SC_TOIE_MASK) | FTM_SC_TOIE_MASK;//habilito la interrupcion de overflow
	NVIC_EnableIRQ(irqEnable[witchFtm]);//habilito la interrupcion de ese periferico

	*/
	// Deshabilitar el periferico para la inicializacion
	FTM0->SC = 		(FTM0->SC 	 & ~FTM_SC_CLKS_MASK) 	| FTM_SC_CLKS(0);

	FTM0->MODE =	(FTM0->MODE  & ~FTM_MODE_FTMEN_MASK)| FTM_MODE_FTMEN_MASK;//pongo el ftmen en 1
	FTM0->SC = 		(FTM0->SC 	 & ~FTM_SC_PS_MASK) 	| FTM_SC_PS(DRIVER_FTM_PSC); //indico que utilizo el sitem clock 50M
	FTM0->SC = 		(FTM0->SC 	 & ~FTM_SC_TOIE_MASK) 	| FTM_SC_TOIE_MASK;//habilito la interrupcion de overflow
	FTM0->MOD = 	(FTM0->MOD 	 & ~FTM_MOD_MOD_MASK)	| FTM_MOD_MOD(DRIVER_FTM_CNTIN_MOD);//seteo el valor al que llega el contador
	FTM0->CNTIN = 	(FTM0->CNTIN & ~FTM_CNTIN_INIT_MASK)| FTM_CNTIN_INIT(DRIVER_FTM_CNTIN_INIT);//seteo el valor inicial del contador

	//Elijo fuente de clock al final, porque al cambiarlo de 0 se prende
	FTM0->SC = 		(FTM0->SC 	 & ~FTM_SC_CLKS_MASK) 	| FTM_SC_CLKS(DRIVER_FTM_CLK_SOURCE); //indico que utilizo el sitme clock 50M

	NVIC_EnableIRQ(FTM0_IRQn);//habilito la interrupcion de ese periferico

	//	FTM0->MODE = 	(FTM0->MODE	 & ~FTM_MODE_INIT_MASK)	| FTM_MODE_INIT_MASK;
}

void tst(void){

}



//void tst(void)
//{}
void FTM0_DriverIRQHandler(void){
	uint8_t overFlowFlag=0;
	overFlowFlag=FTM0->SC & FTM_SC_TOF_MASK;//veo si la interrupcion es de overflow
	if(overFlowFlag){
		FTM0->SC = (FTM0->SC & ~ FTM_SC_TOF_MASK) | FTM_SC_TOF(0);//bajo el flag de la irq
//		FTM0->MOD = 	(FTM0->MOD 	 & ~FTM_MOD_MOD_MASK)	| FTM_MOD_MOD(DRIVER_FTM_CNTIN_MOD);//seteo el valor al que llega el contador
		gpioToggle(PIN_SCK);
	}
}


