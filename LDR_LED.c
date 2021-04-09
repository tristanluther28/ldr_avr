//-----------------------------------------------------------------------------
//
//  LDR_LED.c
//
//  Swallowtail Auto Dim LED Firmware
//  AVR (ATtiny261) Auto Dim LED Firmware
//
//  Copyright (c) 2020 Swallowtail Electronics
//
//  Permission is hereby granted, free of charge, to any person obtaining a
//  copy of this software and associated documentation files (the "Software"),
//  to deal in the Software without restriction, including without limitation
//  the rights to use, copy, modify, merge, publish, distribute, sub-license,
//  and/or sell copies of the Software, and to permit persons to whom the
//  Software is furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in
//  all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
//  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
//  DEALINGS IN THE SOFTWARE.
//
//  Web:    http://tristanluther.com
//  Email:  tristanluther28@gmail.com
//
//-----------------------------------------------------------------------------

/******************** Macros *****************************/

#ifndef F_CPU
#define F_CPU 1000000UL //Set clock speed to 1MHz
#endif

#define BIT_SET(byte, bit) (byte & (1<<bit))

/******************** Includes ***************************/

#include <avr/io.h>

/******************* Globals *****************************/

//Add volatile keyword so the compiler won't optimize these variables out if only used in ISR

/******************** Functions **************************/

//Initialize the PWM Output for OCR1B, set the max value in OCR1B
void PWM_init(){
	//Enable PWM Output B
	TCCR1A = (1<<PWM1B) | (0<<COM1B1) | (1<<COM1B0);
	//Enable Clock Divider/64
	TCCR1B = (0<<CS13)|(1<<CS12)|(1<<CS11)|(1<<CS10);
	//Enable Fast PWM Mode
	TCCR1D = (0<<WGM11)|(0<<WGM10);
	//Use PLL Clock for PWM Generator
	PLLCSR |= (1 << PCKE)|(1 << PLLE);
	//Max PWM value possible set though OCR1C
	OCR1C = 0xFF;
	return; //Return to call point
}

//Initialize the ADC
void ADC_init(){
	//Use VCC for the analog reference voltage, left justify result, only use ADC0 for input
	ADMUX = (0<<REFS1)|(0<<REFS0)|(0<<ADLAR)|(0<<MUX4)|(0<<MUX3)|(0<<MUX2)|(0<<MUX1)|(0<<MUX0);
	//Enable ADC, use for single conversion mode, clk/8 prescaler
	ADCSRA = (1<<ADEN)|(0<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
	//Use ADC0 for input
	ADCSRB = (0<<MUX5);
	return; //Return to call point
}

//Reads the value from the ADC (Battery level)
uint16_t ADC_value(){
	//Pass bit in to get request measurement
	ADCSRA |= (1<<ADSC);
	//Wait for conversion
	while(BIT_SET(ADCSRA, ADSC)) {}
	//Return the left justified result
	return ADC;
}

/******************** Interrupt Service Routines *********/


/******************** Main *******************************/
int main(void)
{
	/* Initialize the I/O Registers */
	/*				-I/O Map-
	 *	Reactive LED: PB3 (OC1B Timer Counter) (1: Output)
	 *  Light Dependent Voltage: PA0 (0: Input)
	 */
	DDRB |= (1<<PB3);
	//Set the default values for outputs to zero and inputs to have pull-up resistors
	PORTB |= (0<<PB3);

	/* Initialize the timer/counter0 (Fast PWM Mode) */
	PWM_init();
	
	/* Initialize the analog input */
	ADC_init();

	/* Storage for the LDR value from the ADC */
	uint16_t LDRvalue = 0;
	
    /* State machine loop */
    while (1) 
    {
		//Get the LDR value
		LDRvalue = ADC_value();
		//Based on the LDR value set the brightness of the LED though OCR1B
		if(LDRvalue < 100){
			OCR1B = 255;
		}
		else if(LDRvalue >= 100 && LDRvalue < 290){
			OCR1B = 180;
		}
		else if(LDRvalue >= 290 && LDRvalue < 400){
			OCR1B = 120;
		}
		else if(LDRvalue >= 400 && LDRvalue < 800){
			OCR1B = 80;
		}
		else if(LDRvalue >= 800 && LDRvalue < 1000){
			OCR1B = 20;
		}
		else{
			OCR1B = 10;
		}
    }
}

