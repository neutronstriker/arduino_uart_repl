/*
 * i2c.cpp
 *
 * Created: 31-05-2015 00:32:00
 *  Author: neutron
 */

#include "i2c.h"

uint16_t timeout_val = 2000; //2 seconds default can be modified later
uint32_t time_stamp=0;

int i2c_init(unsigned long i2c_clk)
{

	unsigned int TWBR_VAL;

	//from i2c clk calculation formula and works only when TWPS is zero

	TWBR_VAL = ((F_CPU/i2c_clk)-16UL)/2UL;

	//this means that the value of TWBR has gone above 255 i.e. it won't fit in
	//an 8 bit register like TWBR  so this is an error so we return -1 to
	//notify the error.
	if(TWBR_VAL > 255)
		return 0;

	TWSR = 0; //clear all the TWPS bits //however Status bits TWS3-7 won't be
	//affected since they are read only bits.

	TWBR = TWBR_VAL;//load value generated from the calculation into TWBR

	TWCR  = (1<<TWEN); //enable the TWI module in AVR
	//note we don't need to set the functionality of SDA and SCL pins
	//(i.e. SDA is PC4 and SCL is PC5 in 28pin AVRs)  because the when
	//we enable TWI module those pins are overriden by TWI module and
	//are in OPENDRAIN mode.

	return TWBR_VAL;//notify the I2C has been Initialised successfully.
}


//A note about the TWINT flag:
/* TWINT is short for TWI interrupt flag, when the TWI unit in AVR completes
an job(one operation) its sets TWINT flag high. So if Global Interrupt flag is
also set along with TWIE(TWI interrupt enable bit) in TWCR then it can trigger
an interrupt.

The TWI module in AVR cannot proceed to do any operation if TWINT flag is set
in TWCR so to do any operation we must clear the TWINT flag by writing ONE to it.

When the TWINT flag is set the TWI module of AVR pulls the SCL line LOW irrespective
of whether it is in MASTER mode of SLAVE mode. There by allowing the SOFTWARE to
respond to the JOB COMPLETION notification.

IF interrupts are not enabled then after Writing appropriate data to respective
TWI registers and clearing the TWINT flag by writing ONE to it. We must keep polling
the TWINT flag in TWCR to know when the Transmission has completed or if any new data
is received or have received a STOP or START condition.

When ever the TWINT flag is set we must check the TWS3-7 status bits in TWSR to know the
status of TWI unit and proceed according to it.

*/

int i2c_start()
{
    //transmit a start condition.
	TWCR = (1<<TWEN) | (1<<TWSTA) | (1<<TWINT); //clear the TWINT flag while doing any operation
    
	time_stamp=millis();
    
    while(!(TWCR & (1<<TWINT))) //wait until operation is complete by polling TWINT flag.
    {
	    if((millis()-time_stamp)>timeout_val)
	    {
		    return 2; //device timed out, take care of error handling in the calling function.
	    }
    }

	if((TWSR & 0xF8) != TW_START) // TW_START is the definition for START condition defined in
		return 0;				//in <util/twi.h> we are masking the first three bits in  TWSR because
		//first two bits are TWPS(prescaler bits) and third bit is reserved.

	return 1; //return 1 means success
}

int i2c_rep_start()
{
	//Repeated start and Start have different status code despite being the same kind of
	//operation they leave a different imprint in the status bits of TWSR

	

	TWCR = (1<<TWEN) | (1<<TWSTA) | (1<<TWINT);
	
	time_stamp=millis();
   	
	while(!(TWCR & (1<<TWINT))) //wait until operation is complete by polling TWINT flag.
	{
		if((millis()-time_stamp)>timeout_val)
		{
			return 2; //device timed out, take care of error handling in the calling function.
		}
	}
	if((TWSR & 0xf8) != TW_REP_START)
		return 0;

	return 1;
}


void i2c_stop()
{
	//transmit a stop condition.
	TWCR = (1<<TWEN) | (1<<TWSTO) | (1<<TWINT);

	//we cannot poll the TWINT flag to check whether the stop condition was transmitted or not

	//And neither can we check the status code register.

	//Another point to keep in mind is that after sending a stop condition we will
	//in any case send a start condition first and in the start condition function
	//we are checking if the start condition was transmitted successfully or not.

	//And also if we transmit a stop condition when we are in control of the bus
	//the connected slave inhibits all operations and stops communication.
	//So we don't need to worry that the slave will may transmit data after sending
	//a stop cmd and it may be lost. But if we send a STOP while the slave is transmitting
	//then it could be an illegal case.
}


unsigned char i2c_transmit(unsigned char data)
{
	
	do{

		TWDR = data;

	}while(TWCR & (1<<TWWC)); //load the data into TWDR and also check if any write collision
	//occured if there is any collision then keep writing the data into TWDR until
	//there is no collision. It is optional as if in all routines we wait until TWINT is high
	//then there is no need of this.

	//Write collision occurs when we try to write data to TWDR when TWINT is low.
	//The TWWC flag is cleared automatically on writing to TWDR when TWINT is high.

	TWCR = (1<<TWINT) | (1<<TWEN); //Clear the TWINT flag and keep I2C unit enabled
	//so that data is transmitted in the next cycle.

	time_stamp=millis();
	
	while(!(TWCR & (1<<TWINT))) //wait until operation is complete by polling TWINT flag.
	{
		if((millis()-time_stamp)>timeout_val)
		{
			return 2; //device timed out, take care of error handling in the calling function.
		}
	}

	return (TWSR & 0xf8); //return the status code
}


unsigned char i2c_receive(unsigned char acknowledge, uint8_t *data)
{
	if(acknowledge) //if it is the last byte then send a NACK
		TWCR = (1<<TWEN) | (1<<TWINT);
	else //if it is not the last byte Transmit a ACK
		TWCR = (1<<TWEN) | (1<<TWEA) | (1<<TWINT);

	time_stamp=millis();
	
	while(!(TWCR & (1<<TWINT))) //wait until operation is complete by polling TWINT flag.
	{
		if((millis()-time_stamp)>timeout_val)
		{
			return 2; //device timed out, take care of error handling in the calling function.
		}
	}
	
	*data = TWDR;
	return 1;
}


unsigned char i2c_tx_sla_r(unsigned char address)
{
	//Transmit the 7 bit address by appending 1 at the end
	return i2c_transmit((address<<1) | (1<<0)); //and return the status code
}

unsigned char i2c_tx_sla_w(unsigned char address)
{
	//Transmit the 7 bit address by appending 0 at the end
	return i2c_transmit(address<<1); //and return the status code
}


uint8_t i2c_write(uint8_t address, uint8_t reg, uint8_t *data, uint8_t bytes)
{
		i2c_start();
		
		i2c_tx_sla_w(address);
		
		i2c_transmit(reg);
		
		for(uint8_t i=0;i<bytes;i++)
		{
			i2c_transmit(data[i]);
		}
	
		i2c_stop();
	
	return 1; //return 2 if timed-out
}


uint8_t i2c_read(uint8_t address, uint8_t reg, uint8_t *data, uint8_t bytes)
{
		i2c_start();
		
		i2c_tx_sla_w(address);
	
		i2c_transmit(reg);
	
		i2c_rep_start();
			
		i2c_tx_sla_r(address);
			
		for(uint8_t i=0;i<bytes;i++)
		{
			if(i!=bytes-1)
			{
				i2c_receive(ACK,&data[i]);
			}
			else
			{
				i2c_receive(NACK,&data[i]);
			}
		}
			
		i2c_stop();
	
	return 1;//return 2 if timed-out	
}

uint8_t i2c_ping( uint8_t address )
{
	uint8_t response;
	i2c_start();
	if (i2c_tx_sla_w(address) == TW_MT_SLA_ACK)		//if the device ACKs on trying to write to that address
	{
		response = 1;
	}
	else
	{
		response = 0;
	}
	i2c_stop();
	
	return response;
	
}

