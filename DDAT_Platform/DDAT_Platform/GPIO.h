/*
 * GPIO.h
 *
 * Created: 30-05-2015 16:32:18
 *  Author: neutron
 */ 


#ifndef GPIO_H_
#define GPIO_H_

#include "pindefines.h"
#include <avr/io.h>

class GPIO{
	
	private:
	
	volatile uint8_t * GPIO_ADD;
	uint8_t pin;
	
	public:
	
	void parsePinMap(uint8_t PORT_PIN);
	
	GPIO(uint8_t GPIO_PIN_NUM);
	
	~GPIO()
	{
		//Destructor
		//nothing to do
	}
	
	private:
	
	void setOutPut();
	//bool isOutput; I could implement state checking of DDR register but then any direct change of DDR
	//anywhere else would make the HIGH and LOW commands render useless
	//so every-time in GPIO.HIGH() and GPIO.LOW() both DDR and PORT register will be written.
	
	public:
	
	void setInput();
	void setInputPullUp();
	void setInputPullDown();
	
	void High();
	void Low();
	void toggle();
	
	bool getState();
	
};




#endif /* GPIO_H_ */