/*Autonomous Control and Navigation of Omni-Drive Vehicle(ODV)*/

#include "tm4c123gh6pm.h"
#include "PLL.h"
#include "UART.h"
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include "fifo.h"
#include "PWM_Init.h"
#include "Encoder_Capture.h"

#define Kp 	5
#define Ki	0.08
#define Kd	0.1

#define FIFO_size 100


void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void WaitForInterrupt(void);  // low power mode

volatile bool disp_flag=0;
volatile bool cap_index[4]= {0,0,0,0};
volatile float set_rpm=200.0;
volatile int queue_count[4]= {0,0,0,0};

volatile float time_queue_1[FIFO_size],time_queue_2[FIFO_size],time_queue_3[FIFO_size],time_queue_4[FIFO_size];
FIFO fifo[4];


volatile unsigned int cap_val_1[2],cap_val_2[2],cap_val_3[2],cap_val_4[2];
volatile float vel_sum_1=1,vel_sum_2=1,vel_sum_3=1,vel_sum_4=1;
volatile int rpm[4], prev_rpm[4];
float eIntg=0, prev_error=0;


void PWM_assign(unsigned int PWM)
{
	PWM0_3_CMPA_R = PWM;
  PWM0_3_CMPB_R = PWM;
}

void PID_Controller(float rpm_error)
{
	float PID_value=0;
	long int pwm = 0;
	PID_value=(Kp*rpm_error)+(Ki*eIntg*0.01)+(Kd*(rpm_error-prev_error)/0.01);
	eIntg+=rpm_error;
	prev_error=rpm_error;
	pwm=(long int)PWM0_3_CMPA_R + PID_value;
	if(pwm>=49998)
		pwm=49998;
	else if(pwm<=0)
		pwm=0;
	PWM_assign(pwm);
}



void Timer0_Init(unsigned long period)
{
  SYSCTL_RCGCTIMER_R |= 0x01;   // 0) activate TIMER0  
  TIMER0_CTL_R = 0x00000000;    // 1) disable TIMER0A during setup
  TIMER0_CFG_R = 0x00000000;    // 2) configure for 32-bit mode
  TIMER0_TAMR_R = 0x00000002;   // 3) configure for periodic mode, default down-count settings
  TIMER0_TAILR_R = period-1;    // 4) reload value
  TIMER0_TAPR_R = 0;            // 5) bus clock resolution
  TIMER0_ICR_R = 0x00000001;    // 6) clear TIMER0A timeout flag
  TIMER0_IMR_R = 0x00000001;    // 7) arm timeout interrupt
  NVIC_PRI4_R = (NVIC_PRI4_R&0x00FFFFFF)|0x80000000; // 8) priority 4
	// interrupts enabled in the main program after all devices initialized
	// vector number 35, interrupt number 19
  NVIC_EN0_R = 1<<19;           // 9) enable IRQ 19 in NVIC
  TIMER0_CTL_R = 0x00000001;    // 10) enable TIMER0A
}

void Timer0A_Handler()
{
	TIMER0_ICR_R = TIMER_ICR_TATOCINT;// acknowledge TIMER0A timeout
	disp_flag=1;		
}


int LPF(int prev, int curr, float weight)
{
	return weight*curr + (1-weight)*prev;
}

int main()
{
	fifo_init(&fifo[0], FIFO_size, (float*)time_queue_1);
	fifo_init(&fifo[1], FIFO_size, (float*)time_queue_2);
	fifo_init(&fifo[2], FIFO_size, (float*)time_queue_3);
	fifo_init(&fifo[3], FIFO_size, (float*)time_queue_4);
	UART0_Init();
	PLL_Init();
	Timer0_Init(800000*4);
	WideTimer1_Init();
	WideTimer3_Init();
	
	Motor1_PWM_Init();
	Motor2_PWM_Init();
	Motor3_PWM_Init();
	Motor4_PWM_Init();
	
	EnableInterrupts();
	
	while(1)
	{	
		if(disp_flag)
		{
			disp_flag=0;
			rpm[0] = (6.0f)/(90*vel_sum_1);
			rpm[1] = (6.0f)/(90*vel_sum_2);
			rpm[2] = (6.0f)/(90*vel_sum_3);
			rpm[3] = (6.0f)/(90*vel_sum_4);
			prev_rpm[0]=rpm[0];
			prev_rpm[1]=rpm[1];
			prev_rpm[2]=rpm[2];
			prev_rpm[3]=rpm[3];
			//rpm=LPF(prev_rpm, rpm, 0.9);
			UART_OutUDec(rpm[2],0);
			Print_Space(2,0);
			//PID_Controller(rpm - set_rpm);
			//vel_sum=0;
			//edge_counter=0;
			/*UART0_OutChar(0xAB);
			UART0_OutChar(0xCD);
			UART0_OutChar(0x02);
			UART0_OutChar(0x00);
			UART0_OutChar(rpm & 0x00FF);
			UART0_OutChar((rpm & 0xFF00)>>8);*/
		}		
	}
}



void WideTimer1A_Handler()
{
		float temp=0;
		WTIMER1_ICR_R = TIMER_ICR_CAECINT;	// acknowledge TIMER0A capture event
    cap_val_1[0]= WTIMER1_TAR_R;  // execute user task
		if(cap_index[0])
		{
			temp=(cap_val_1[1]-cap_val_1[0])*12.5*0.000000001;
			//vel_sum +=(cap_val[1]-cap_val[0])*12.5*0.000000001;
			//edge_counter++;
			if(queue_count[0]<FIFO_size)
			{
				queue_count[0]++;
				vel_sum_1=(vel_sum_1*(queue_count[0]-1) + temp)/(queue_count[0]);
				fifo_put(&fifo[0], temp);
			}else if(queue_count[0]==FIFO_size)
			{
				vel_sum_1 +=((temp - fifo_get(&fifo[0]))/(FIFO_size*1.0f)); 
				fifo_put(&fifo[0],temp);
			}
			//UART_OutUDec(vel_sum*1000,0);
			//Print_Space(2,0);
		}else cap_index[0]=1;
		cap_val_1[1]=cap_val_1[0];
}


void WideTimer1B_Handler()
{
		float temp=0;
		WTIMER1_ICR_R = TIMER_ICR_CBECINT;	// acknowledge TIMER0A capture event
    cap_val_2[0]= WTIMER1_TBR_R;  // execute user task
		if(cap_index[1])
		{
			temp=(cap_val_2[1]-cap_val_2[0])*12.5*0.000000001;
			//vel_sum +=(cap_val[1]-cap_val[0])*12.5*0.000000001;
			//edge_counter++;
			if(queue_count[1]<FIFO_size)
			{
				queue_count[1]++;
				vel_sum_2=(vel_sum_2*(queue_count[1]-1) + temp)/(queue_count[1]);
				fifo_put(&fifo[1], temp);
			}else if(queue_count[1]==FIFO_size)
			{
				vel_sum_2 +=((temp - fifo_get(&fifo[1]))/(FIFO_size*1.0f)); 
				fifo_put(&fifo[1],temp);
			}
			//UART_OutUDec(vel_sum*1000,0);
			//Print_Space(2,0);
		}else cap_index[1]=1;
		cap_val_2[1]=cap_val_2[0];
}

void WideTimer3A_Handler()
{
		float temp=0;
		WTIMER3_ICR_R = TIMER_ICR_CAECINT;	// acknowledge TIMER0A capture event
    cap_val_3[0]= WTIMER3_TAR_R;  // execute user task
		if(cap_index[2])
		{
			temp=(cap_val_3[1]-cap_val_3[0])*12.5*0.000000001;
			//vel_sum +=(cap_val[1]-cap_val[0])*12.5*0.000000001;
			//edge_counter++;
			if(queue_count[2]<FIFO_size)
			{
				queue_count[2]++;
				vel_sum_3=(vel_sum_3*(queue_count[2]-1) + temp)/(queue_count[2]);
				fifo_put(&fifo[2], temp);
			}else if(queue_count[2]==FIFO_size)
			{
				vel_sum_3 +=((temp - fifo_get(&fifo[2]))/(FIFO_size*1.0f)); 
				fifo_put(&fifo[2],temp);
			}
			//UART_OutUDec(vel_sum*1000,0);
			//Print_Space(2,0);
		}else cap_index[2]=1;
		cap_val_3[1]=cap_val_3[0];
}


void WideTimer3B_Handler()
{
		float temp=0;
		WTIMER3_ICR_R = TIMER_ICR_CBECINT;	// acknowledge TIMER0A capture event
    cap_val_4[0]= WTIMER3_TBR_R;  // execute user task
		if(cap_index[3])
		{
			temp=(cap_val_4[1]-cap_val_4[0])*12.5*0.000000001;
			//vel_sum +=(cap_val[1]-cap_val[0])*12.5*0.000000001;
			//edge_counter++;
			if(queue_count[3]<FIFO_size)
			{
				queue_count[3]++;
				vel_sum_4=(vel_sum_4*(queue_count[3]-1) + temp)/(queue_count[3]);
				fifo_put(&fifo[3], temp);
			}else if(queue_count[3]==FIFO_size)
			{
				vel_sum_4 +=((temp - fifo_get(&fifo[3]))/(FIFO_size*1.0f)); 
				fifo_put(&fifo[3],temp);
			}
			//UART_OutUDec(vel_sum*1000,0);
			//Print_Space(2,0);
		}else cap_index[3]=1;
		cap_val_4[1]=cap_val_4[0];
}


