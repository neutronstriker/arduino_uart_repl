/*
 * GPIO.cpp
 *
 * Created: 30-05-2015 16:33:11
 *  Author: neutron
 */ 
#include "GPIO.h"

void GPIO::parsePinMap(uint8_t PORT_PIN)
{
	if(PORT_PIN>=0 && PORT_PIN <= 7)
	{
		GPIO_ADD =  &PORTD;
		pin = PORT_PIN;
	}
	
	else if(PORT_PIN>=8 && PORT_PIN<=13)
	{
		GPIO_ADD = &PORTB;
		pin = PORT_PIN-8;
	}
	
	else if(PORT_PIN>= A0 && PORT_PIN<= A5)
	{
		GPIO_ADD = &PORTC;
		pin = PORT_PIN-14;
	}
}

void GPIO::setOutPut()
{
	*(GPIO_ADD-1) |= (1<<pin); //DDR address is always PORT address-1 for respective PORTs
}

void GPIO::High()
{
	/*	
	setOutPut();
	*(GPIO_ADD) |= (1<<pin);
	*/
	*(GPIO_ADD) |= (1<<pin);
	setOutPut();
}

void GPIO::Low()
{
	/*
	
	//the sequence of operation has been corrected other wise cause a small glitch while trying to operate in open drain 
	//mode we don't want the situation when if earlier it was set to input high and then if you call setOutput() first
	//before setting the port pin value then it will cause a small glitch depending on the time taken for the two calls
	//to complete
	
	setOutPut();
	*GPIO_ADD &= ~(1<<pin);
	
	*/
	*GPIO_ADD &= ~(1<<pin);
	setOutPut();
}

void GPIO::toggle()
{
	setOutPut();
	*GPIO_ADD ^= (1<<pin);
}

void GPIO::setInput()
{
	*(GPIO_ADD-1) &= ~(1<<pin);
}

void GPIO::setInputPullDown()
{
	*(GPIO_ADD-1) &= ~(1<<pin);
	*(GPIO_ADD) &= ~(1<<pin);
}

void GPIO::setInputPullUp()
{
	*(GPIO_ADD-1) &= ~(1<<pin);
	*(GPIO_ADD) |= (1<<pin);
}

bool GPIO::getState()
{
	return (*(GPIO_ADD-2) & (1<<pin)); //PIN register address is always PORT_reg_address-2 for respective ports
}


GPIO::GPIO(uint8_t GPIO_PIN_NUM)
{
	parsePinMap(GPIO_PIN_NUM);
}