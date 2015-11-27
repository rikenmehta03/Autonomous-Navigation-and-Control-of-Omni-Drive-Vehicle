#include "tm4c123gh6pm.h"
#include "PWM_Init.h"

void Motor4_PWM_Init(void)
{  
    SYSCTL_RCGCPWM_R |= 0x00000001; // activate PWM0 clock
    SYSCTL_RCGCGPIO_R |= (1<<PORTC); // activate port D clock  (PD0, PD1)
    while((SYSCTL_PRGPIO_R&(1<<PORTC)) == 0){}; // wait until port D is ready
    GPIO_PORTC_AFSEL_R |= (1<<PC4)|(1<<PC5);  // enable PD0 and PD1 pin alternative functionality
    GPIO_PORTC_PCTL_R = (GPIO_PORTC_PCTL_R&0xFF00FFFF)+0x00440000;  // replace porte to portD
    GPIO_PORTC_AMSEL_R &= ~((1<<PC5)|(1<<PC4)); // disable aternative functionality PD0 and PD1
    GPIO_PORTC_DEN_R |= (1<<PC4)|(1<<PC5); // enable i/o on PD0 and PD1
    SYSCTL_RCC_R =((SYSCTL_RCC_R&0xFF00FFFF)|0x00140000); // use USEPWMDIV and set divider PWMDIV to divide by 8 

		PWM0_3_CTL_R &= ~0x00000002;  
    PWM0_3_GENA_R |= 0x0000008C; 
    PWM0_3_GENB_R |= 0x0000080C; 

    PWM0_3_LOAD_R = 49999; 
			
    PWM0_3_CMPA_R = 24999;
    PWM0_3_CMPB_R =  24999;
    
		PWM0_3_CTL_R |= 0x00000001; // start PWM Generator 3 timers
    PWM0_ENABLE_R |= (1<<6)|(1<<7); // enable PWM output
}

void Motor2_PWM_Init(void)
{  
    SYSCTL_RCGCPWM_R |= 0x00000001; // activate PWM0 clock
    SYSCTL_RCGCGPIO_R |= (1<<PORTB); // activate port B clock  (PB4, PB5)
    while((SYSCTL_PRGPIO_R&(1<<PORTB)) == 0){}; // wait until port B is ready
    GPIO_PORTB_AFSEL_R |= (1<<PB4)|(1<<PB5);  // enable PB4 and PB5 pin alternative functionality
    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFF00FFFF)+0x00440000;  // replace porte to portB
    GPIO_PORTB_AMSEL_R &= ~((1<<PB4)|(1<<PB5)); // disable aternative functionality PB4 and PB5
    GPIO_PORTB_DEN_R |= (1<<PB4)|(1<<PB5); // enable i/o on PB4 and PB5
		SYSCTL_RCC_R =((SYSCTL_RCC_R&0xFF00FFFF)|0x00140000); // use USEPWMDIV and set divider PWMDIV to divide by 8 

		PWM0_1_CTL_R &= ~0x00000002;  
    PWM0_1_GENA_R |= 0x0000008C; 
    PWM0_1_GENB_R |= 0x0000080C; 

    PWM0_1_LOAD_R = 49999; 
			
    PWM0_1_CMPA_R = 24999;
    PWM0_1_CMPB_R = 24999;
    
		PWM0_1_CTL_R |= 0x00000001; // start PWM Generator 3 timers
    PWM0_ENABLE_R |= (1<<3)|(1<<2); // enable PWM output
}

void Motor1_PWM_Init(void)
{  
    SYSCTL_RCGCPWM_R |= 0x00000001; // activate PWM0 clock
    SYSCTL_RCGCGPIO_R |= (1<<PORTB); // activate port B clock  (PB6, PB7)
    while((SYSCTL_PRGPIO_R&(1<<PORTB)) == 0){}; // wait until port B is ready
    GPIO_PORTB_AFSEL_R |= (1<<PB6)|(1<<PB7);  // enable PB6 and PB7 pin alternative functionality
    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0x00FFFFFF)+0x44000000;  // replace porte to portB
    GPIO_PORTB_AMSEL_R &= ~((1<<PB6)|(1<<PB7)); // disable aternative functionality PB6 and PB7
    GPIO_PORTB_DEN_R |= (1<<PB6)|(1<<PB7); // enable i/o on PB6 and PB7
    SYSCTL_RCC_R =((SYSCTL_RCC_R&0xFF00FFFF)|0x00140000); // use USEPWMDIV and set divider PWMDIV to divide by 8 

		PWM0_0_CTL_R &= ~0x00000002;  
    PWM0_0_GENA_R |= 0x0000008C; 
    PWM0_0_GENB_R |= 0x0000080C; 

    PWM0_0_LOAD_R = 49999; 
			
    PWM0_0_CMPA_R = 10000;
    PWM0_0_CMPB_R = 24999;
    
		PWM0_0_CTL_R |= 0x00000001; // start PWM Generator 3 timers
    PWM0_ENABLE_R |= (1<<0)|(1<<1); // enable PWM output
}

void Motor3_PWM_Init(void)
{  
    SYSCTL_RCGCPWM_R |= 0x00000001; // activate PWM0 clock
    SYSCTL_RCGCGPIO_R |= (1<<PORTE); // activate port E clock  (PE4, PE5)
    while((SYSCTL_PRGPIO_R&(1<<PORTE)) == 0){}; // wait until port E is ready
    GPIO_PORTE_AFSEL_R |= (1<<PE4)|(1<<PE5);  // enable PE5 and PE5 pin alternative functionality
    GPIO_PORTE_PCTL_R = (GPIO_PORTE_PCTL_R&0xFF00FFFF)+0x00440000;  // replace porte to portE
    GPIO_PORTE_AMSEL_R &= ~((1<<PE4)|(1<<PE5)); // disable aternative functionality PE4 and PE5
    GPIO_PORTE_DEN_R |= (1<<PE4)|(1<<PE5); // enable i/o on PE4 and PE5
    SYSCTL_RCC_R =((SYSCTL_RCC_R&0xFF00FFFF)|0x00140000); // use USEPWMDIV and set divider PWMDIV to divide by 8 

		PWM0_2_CTL_R &= ~0x00000002;  
    PWM0_2_GENA_R |= 0x0000008C; 
    PWM0_2_GENB_R |= 0x0000080C; 

    PWM0_2_LOAD_R = 49999; 
			
    PWM0_2_CMPA_R = 24999;
    PWM0_2_CMPB_R = 24999;
    
		PWM0_2_CTL_R |= 0x00000001; // start PWM Generator 3 timers
    PWM0_ENABLE_R |= (1<<4)|(1<<5); // enable PWM output
}
