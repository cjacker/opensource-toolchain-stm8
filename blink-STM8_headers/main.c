/**********************
  Simple blink project w/o interrupts
   
  Functionality:
    - blink LED w/o ISR. Mainly for testing toolchain
    - pass port structure to function
**********************/

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include "STM8S208MB.h"
#define LED_PORT   sfr_PORTD
#define LED_PIN    PIN0



// toggle specified pin. Pass port struct as pointer
void toggle_pin(PORT_t *port, uint8_t pin) {
  
  port->ODR.byte ^= pin;
  
} // toggle_pin



/////////////////
//    main routine
/////////////////
void main (void) {

  uint32_t  i;
    
  // switch to 16MHz (default is 2MHz)
  sfr_CLK.CKDIVR.byte = 0x00;
    
  // configure LED pin as output
  LED_PORT.DDR.byte = LED_PIN;    // input(=0) or output(=1)
  LED_PORT.CR1.byte = LED_PIN;    // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
  LED_PORT.CR2.byte = LED_PIN;    // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope
  

  // main loop
  while(1) {
	
    // toggle LED. Pass port struct as pointer
    toggle_pin(&LED_PORT, LED_PIN);
    
    // simple wait ~1000ms @ 16MHz
    for (i=0; i<600000L; i++)
      NOP();
    
  } // main loop

} // main

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
