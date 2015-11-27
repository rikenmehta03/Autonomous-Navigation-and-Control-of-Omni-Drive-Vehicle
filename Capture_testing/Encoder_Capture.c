#include "tm4c123gh6pm.h"
#include "Encoder_Capture.h"

void WideTimer1_Init()
{
	SYSCTL_RCGC2_R |= (1<<PORTC);
	GPIO_PORTC_DEN_R |= (1<<PC6)|(1<<PC7);
	GPIO_PORTC_ODR_R &= ~((1<<PC6)|(1<<PC7));
	GPIO_PORTC_PCTL_R = (GPIO_PORTC_PCTL_R & 0x00FFFFFF)| 0x77000000;
	GPIO_PORTC_AFSEL_R |= (1<<PC6)|(1<<PC7);
	
	SYSCTL_RCGCWTIMER_R |= (1<<1);   // 0) activate Wide TIMER0 clock
	WTIMER1_CTL_R = 0x00000000;    // 1) disable Wide TIMER0A during setup
	WTIMER1_CFG_R = 0x00000004;    // 2) configure for Input-Edge mode
	WTIMER1_TAMR_R= 0x00000007;		 // 3) Input Rising Edge Timing mode
	WTIMER1_TBMR_R= 0x00000007;		 // 3) Input Rising Edge Timing mode
	WTIMER1_CTL_R |= 0x00000C0C; // 4) Configure for timing mode
	WTIMER1_TAILR_R = 0xFFFFFFFF;  // 5) Reload value
	WTIMER1_TAPR_R = 0;
	WTIMER1_TBILR_R = 0xFFFFFFFF;  // 5) Reload value
	WTIMER1_TBPR_R = 0;
	WTIMER1_ICR_R = 0x00000404;
	WTIMER1_IMR_R = 0x00000404;    // 6) Timer capture event match interrupt
	NVIC_PRI24_R = (NVIC_PRI24_R&0xFFFF0000)|0x00000000; // 7) priority 0 (Highest)
	
	NVIC_EN3_R|=(1<<0)|(1<<1);  // 8) Enable WideTimer0A_Handler
	WTIMER1_CTL_R|= (0x00000101);	 //	9) Timer Enable
}

void WideTimer3_Init()
{
	SYSCTL_RCGC2_R |= (1<<PORTD);
	GPIO_PORTD_DEN_R |= (1<<PD2)|(1<<PD3);
	GPIO_PORTD_ODR_R &= ~((1<<PD2)|(1<<PD3));
	GPIO_PORTD_PCTL_R = (GPIO_PORTD_PCTL_R & 0xFFFF00FF)| 0x00007700;
	GPIO_PORTD_AFSEL_R |= (1<<PD2)|(1<<PD3);
	
	SYSCTL_RCGCWTIMER_R |= (1<<3);   // 0) activate Wide TIMER0 clock
	WTIMER3_CTL_R = 0x00000000;    // 1) disable Wide TIMER0A during setup
	WTIMER3_CFG_R = 0x00000004;    // 2) configure for Input-Edge mode
	WTIMER3_TAMR_R= 0x00000007;		 // 3) Input Rising Edge Timing mode
	WTIMER3_TBMR_R= 0x00000007;		 // 3) Input Rising Edge Timing mode
	WTIMER3_CTL_R |= 0x00000C0C; // 4) Configure for timing mode
	WTIMER3_TAILR_R = 0xFFFFFFFF;  // 5) Reload value
	WTIMER3_TAPR_R = 0;
	WTIMER3_TBILR_R = 0xFFFFFFFF;  // 5) Reload value
	WTIMER3_TBPR_R = 0;
	WTIMER3_ICR_R = 0x00000404;
	WTIMER3_IMR_R = 0x00000404;    // 6) Timer capture event match interrupt
	NVIC_PRI25_R = (NVIC_PRI25_R&0xFFFF0000)|0x00000000; // 7) priority 0 (Highest)
	
	NVIC_EN3_R|=(1<<4)|(1<<5);  // 8) Enable WideTimer0A_Handler
	WTIMER3_CTL_R|= (0x00000101);	 //	9) Timer Enable
}

