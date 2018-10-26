/*
 * i2c.h
 *
 * Created: 31-05-2015 00:30:02
 *  Author: neutron
 */


#ifndef I2C_H_
#define I2C_H_

#include "Arduino.h"

#include<util/twi.h> //contains defines for TWI status codes

#define ACK 	0
#define NACK 	1

#define I2C_OK 		1
#define I2C_ERROR 	0
#define I2C_TIMEOUT 2

int i2c_init(unsigned long i2c_clk);
int i2c_start();
int i2c_rep_start();
void i2c_stop();
unsigned char i2c_transmit(unsigned char data);
unsigned char i2c_receive(unsigned char acknowledge, uint8_t *data);
unsigned char i2c_tx_sla_r(unsigned char address);
unsigned char i2c_tx_sla_w(unsigned char address);


uint8_t i2c_write(uint8_t address, uint8_t reg, uint8_t *data, uint8_t bytes); 
uint8_t i2c_read(uint8_t address, uint8_t reg, uint8_t *data, uint8_t bytes); 
uint8_t i2c_ping(uint8_t address);

//some a timeout based function would be good.
#endif /* I2C_H_ */
