#include "msp430g2553.h"
#include "math.h"

#define ENCODER_A BIT5
#define ENCODER_B BIT7

float Kp = 10.65;
float Ki = 0.026;
float Kd = 57.53;

float Kp_position = 7;
float Ki_position = 0.0;
float Kd_position = 0;

int index = 0,i = 0, position[2] = {0};
float error[2] = {0}, setpoint = 506, integral = 0, differential = 0, position_integral = 0, position_differential = 0, position_bias[2] = {0};
int output = 0, position_output =0;

void PWM_init(void)
{
  TA1CCR0 = 255;  // PWM Period
  TA1CCTL1 = OUTMOD_7; // CCR1 reset/set
  TA1CCTL2 = OUTMOD_7;
  TA1CTL = TASSEL_2 + MC_1; // SMCLK, up mode
}

void IO_init()
{
   P1DIR  = 0x4f;
   P1SEL  = 0x10;
   P1REN  = 0x20;
   P1OUT  = 0x20;
   
   P1IE  |= ENCODER_A;             // Enable the interrupts on P1.5
   P1IES  = 0x00;                  // Low/Hi edge on P1.5
   P1IFG &= ~ENCODER_A;            // Clear the interrupt flag on P1.5
   
   P2DIR  = 0xff;
   P2OUT  &= 0x00; 
   P2SEL |= BIT1; // P2.1 TA1/2 options
}

void ADC_init(void)
{
   ADC10CTL0 = ADC10ON + REFON + REF2_5V;
   ADC10CTL1 = ADC10DIV2 + INCH3;
   ADC10AE0  = INCH3;
   ADC10CTL0|= ENC + ADC10SC;
}

void PWM_output(int power)
{
    if(power > 0)
    {
        P1OUT = 0x04;
        TA1CCR1 = power;  
    }
    else
    {
        P1OUT = 0x08;
        TA1CCR1 = -power;
    }
}


void main(void)
{
  // Stop watchdog timer to prevent time out reset
  WDTCTL = WDTPW | WDTHOLD;
  
  ADC_init(); 
  PWM_init();//initialize
  IO_init();
  
  __enable_interrupt();
   
  for(;;)
  { //code  
    ADC10CTL0 |= ENC + ADC10SC;
    
    error[0]   = error[1];
    error[1]   = ADC10MEM - setpoint;   
    integral  += error[1];
    differential   = error[1] - error[0];
    
    position_output = 0;
    i++;
    if(i == 5)
    {
        position_bias[1] = position[1] - position[0];  
        position[0] = position[1];
        position_integral += position_bias[1];
        position_differential = position_bias[1] - position_bias[0];
        position_bias[0] = position_bias[1];
        position_output = Kp_position*position_bias[1] + Ki_position*position_integral + Kd_position*position_differential;
        
        i=0;
    }
    
    output = Kp*error[1] + Ki*integral + Kd*differential - position_output;
    
    PWM_output(output);
  }
}

#pragma vector=PORT1_VECTOR
__interrupt void Port_1 ( void )
{
   if(P1IN & ENCODER_B)
     position[1]--;
   else
     position[1]++;
   
   P1IFG &= ~ENCODER_A;             // Clear interrupt flag on P1.5
}