
 
/* Includes ------------------------------------------------------------------*/ 
#include "iostm8s103f3.h" 
#include <intrinsics.h>
int data[20];
int direction = 0;
int direction_ok = 0;
int data_cur_pos = 0;
signed int whell1_count = 0;
int target_done = 0;

#pragma vector = TIM1_OVR_UIF_vector //  Timer 2 Overflow handler.
__interrupt void TIM1_UPD_OVF_IRQHandler(void){
 
}
#pragma vector = UART1_R_RXNE_vector
__interrupt void UART1_IRQHandler(void){
  if(data_cur_pos > 3) data_cur_pos = 0;
  data[ data_cur_pos] = UART1_DR;
  data_cur_pos++;
  target_done=0;
  PC_ODR_bit.ODR6 = !PC_ODR_bit.ODR6;
  //send the byte to circular buffer;
}
#pragma vector = 0x07 //PortC Interrupt (All pins)
__interrupt void EXTI_PORTC_IRQHandler(void){

}
#pragma vector = 0x05 //PortA Interrupt (All pins)
__interrupt void EXTI_PORTA_IRQHandler(void){

  if(!direction_ok){
    direction = 1;
    direction_ok = 1;
  }
  whell1_count+=direction;
}




void InitialiseUART(){       
  unsigned char tmp = UART1_SR;
  tmp = UART1_DR;
  //  Reset the UART registers to the reset values.
  UART1_CR1 = 0;
  UART1_CR2 = 0;
  UART1_CR4 = 0;
  UART1_CR3 = 0;
  UART1_CR5 = 0;
  UART1_GTR = 0;
  UART1_PSCR = 0;
  //  Set up the port to 14400,n,8,2.
  UART1_CR1_M    = 0;         //  8 Data bits.
  UART1_CR1_PCEN = 0;         //  Disable parity.
  UART1_CR3      = 0x20;      //  2 stop bits     
   UART1_BRR2 = 0x0a;      //  Set the baud rate registers to 115200 baud
  UART1_BRR1 = 0x08;      //  based upon a 16 MHz system clock.
  //  Disable the transmitter and receiver.
  UART1_CR2_TEN   = 0;        //  Disable transmit.
  UART1_CR2_REN   = 0;        //  Disable receive.
  //  Set the clock polarity, clock phase and last bit clock pulse.
  UART1_CR3_CPOL  = 0;
  UART1_CR3_CPHA  = 0;
  UART1_CR3_LBCL  = 0;
  //  Set the Receive Interrupt RM0016 p358,362
  UART1_CR2_TIEN  = 0;     // Transmitter interrupt enable
  UART1_CR2_TCIEN = 0;     // Transmission complete interrupt enable
  UART1_CR2_RIEN  = 1;     //  Receiver interrupt enable
  UART1_CR2_ILIEN = 0;     //  IDLE Line interrupt enable

  //  Turn on the UART transmit, receive and the UART clock.
  UART1_CR2_TEN    = 1;
  UART1_CR2_REN    = 1;
  UART1_CR1_PIEN   = 0;
  UART1_CR4_LBDIEN = 0;
}

void InitTimer2_PWM(void){
 TIM2_PSCR = 0x03;   
 TIM2_ARRH = (unsigned char)(999 >> 8); 
 TIM2_ARRL = (unsigned char)999;   

 TIM2_CCER1 = 0x00;  // Disable the Channels 1-2
 TIM2_CCER1 = 0x33;  // Enable the Channel 1-2 & Low Polarity
 TIM2_CCER1_bit.CC1E = 0;

 TIM2_CCMR2 = 0x78;  // PWM Mode2(CH2) - Preload  Enabled
 TIM2_CR1  |= 0x80;  // AutoReload aktif durumda.
 TIM2_CR1  |= 0x01;  // Timer çalismaya basliyor.
}

void InitialiseSystemClock(){
    CLK_ICKR = 0;                       //  Reset the Internal Clock Register.
    CLK_ICKR_HSIEN = 1;                 //  Enable the HSI.
    CLK_ECKR = 0;                       //  Disable the external clock.
    while (CLK_ICKR_HSIRDY == 0);       //  Wait for the HSI to be ready for use.
    CLK_CKDIVR = 0;                     //  Ensure the clocks are running at full speed.
    CLK_PCKENR1 = 0xff;                 //  Enable all peripheral clocks.
    CLK_PCKENR2 = 0xff;                 //  Ditto.
    CLK_CCOR = 0;                       //  Turn off CCO.
    CLK_HSITRIMR = 0;                   //  Turn off any HSIU trimming.
    CLK_SWIMCCR = 0;                    //  Set SWIM to run at clock / 2.
    CLK_SWR = 0xe1;                     //  Use HSI as the clock source.
    CLK_SWCR = 0;                       //  Reset the clock switch control register.
    CLK_SWCR_SWEN = 1;                  //  Enable switching.
    while (CLK_SWCR_SWBSY != 0);        //  Pause while the clock switch is busy.
}

void motors_init(){
  //Motor 1
  PC_DDR = 0xEE; 
  PC_CR1 = 0xEE;
  PC_CR2 = 0xFF;
  PC_ODR= 0;
  InitTimer2_PWM();
  //Motor 2
  PD_DDR = 0xFB; 
  PD_CR1 = 0x00; 
  PD_CR2 = 0x00; 
  PD_ODR= 0;
}
void sensors_init(){
  //Sensor 1 (pin already setted up by motors_init)
  EXTI_CR1_bit.PCIS = 2;
  EXTI_CR2_bit.TLIS = 0;
  //Sensor 2
  PA_DDR = 0x00;
  PA_CR1 = 0xFF;
  PA_CR2 = 0xFF;
  EXTI_CR1_bit.PAIS = 2;
  EXTI_CR2_bit.TLIS = 0;
}
void motor1_move( byte dir /* 1 : forward, 0 : backward*/, unsigned char speed /* 0-999 */){
  if(speed > 999) speed = 999;
  
  TIM2_CCR1H = (speed >> 8); 
  TIM2_CCR1L = speed;
  TIM2_CCR2H = TIM2_CCR1H;
  TIM2_CCR2L = TIM2_CCR1L;

  TIM2_CCER1_bit.CC1E = (dir) ? dir : 0;
  TIM2_CCER1_bit.CC2E = (!dir) ? dir : 0;
}
/** 
  * @brief  Main program. 
  * @param  None 
  * @retval None 
  */ 
void main(void) { 
  InitialiseSystemClock();
  __disable_interrupt();

  motors_init();
  sensors_init();
  InitialiseUART();

  __enable_interrupt();
 
  while(whell1_count < 50){
    __disable_interrupt();
    motor1_move(1,999);
    __enable_interrupt();
  }

  while (1) {

  }  
} 
