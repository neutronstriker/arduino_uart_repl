/*
 * config.h
 *
 * Created: 09-03-2016 15:20:54
 *  Author: neutron
 
 I have defined some functions directly in .h files (like UART, DS etc) because of which I am not able to
 include all .h files in config.h, because linker error happens by multiple definition instances.
 
 So the solution is that for such files I should move them to respective .c files and keep only def's in 
 .h file, But this will break compatibility with many other projects, but it won't be a big issue
 during next compile I will have to add the respective .c file or the lib to that project.
 */ 


#ifndef CONFIG_H_
#define CONFIG_H_

//#define PRINT_DEBUG_MSG //if defined all serial debug msgs will be printed, however this could 
						//cause problem for software on PC which acquire data via uart by issuing certain
						//cmds, because debug msg can interfere in the flow of output data.

#define SERIAL_ECHO		//this will enable hardware serial ECHO on console, but will quite weird if console software also has one.

//extern unsigned char ate_enabled;		//defined in the main file. cause we cant define a variable in header file., now this function is 
//integrated into Serial Class.

#define SERIAL_CR_DETECT		//this will enable experimental feature of detecting carriage return in serial driver itself

#define SERIAL_REMOVE_SPACE		//this will remove spaces from Serial Data received, So that I can form cmd structures like
								//i2cwrite -d 0x50 -r 1 -b 3 -u:0x50,0x32,0x42,0x33: and it would be interpreted as
								//i2cwrite-d0x50-r1-b4-u:0x50,0x32,0x42,0x33:, so that it would fit in the serial buffer
								//otherwise white spaces would clog it up. the end colon indicates commands is complete.
								//and valid and before that anyway we will count data bytes number matches specified value
								//i.e. here -b4, and 0x before data is optional. And the command syntax has to be like this
								//we don't have processing power to interpret random order of options.
								//we can make it more simpler though if possible.
								//and maximum bytes at a go would be around 4 to 8. We can make it 8 by removing 0x.
								//I will write my own method to interpret the characters to hex code. So we will use
								//hex code through out here. It will be easier in the fact that it will be always 2 digits
								//and we can easily convert serial ascii into packed byte.
								//there will be command to disable and enable removing spaces as a provision.
								//by default spaces enabled.
								
#define SERIAL_SNOOP_ENABLE		//this will enable a MOD where we will have three options Serial SNoop enable()
								//Serial_snoop_exit() and serial disable (this last one is easy simply call Serial.end())
								//Serial_enter_snoop() will disable TX pin and work as read-only mode.
								//So then we can use the on-board bluetooth for communication with serial devices and 
								//at the same time the DDAT can also interpret data received and we can use to control GPIO
								//or do something else. This will usefull when we want to control another serial device
								//but at the same time perform some logic function on that device.
								//Serial_snoop_exit() will simple flush the existing TX buffer data from the ring buffer
								//and enable TX. We have take of disabling all Serial_TX stuff while doing this like UART_TX_IRQ also.
								//REMEMBER THAT IN SNOOP MODE TO CONNECT TO ANOTHER UART DEVICE USING DDAT BLUETOOTH
								//THE CONNECTION SHOULD BE LIKE BL_RX(NANO_TX)->DEV_TX & BL_TX(NANO_RX)->DEV_RX
								//SO NANO CANNOT READ FROM OTHER DEVICE UNLESS YOU CREATE A SOFT SERIAL AND CONNECT THAT NANO_SOFT_RX->DEV_TX.
								//BUT NANO CAN INTERPRET RECEIVED COMMANDS FROM BL AND TAKE ACTION
								
								//still I am facing certain issues with snoop cmd. Garbage data is going to other side after activating
								//snoop doesn't make any sense because that should have nothing to do with the program


extern unsigned char backspace_det;
#define BACKSPACE_ASCII		127
#define	SERIAL_BACKSPACE_DETECT		//putty uses 127 (dec) as ascii code for backspace as oposed to 8 (dec) used in IBM PC.
									//this enables the section which will make direct command typing more feasible so that 
									//we can backspace when error is typed.
									//But we need to keep in mind that since UART receive buffer size is only 64 so if more than
									//64 chars are typed then only 64 go in and others are not stored. So when tailRemove() is called
									//it removes already stored character from the buffer not the one from the screen and data present
									//on the screen is not what is present in the buffer.
									//so I have implemented even that checking which I wrote in the above line using a var called overFlow_count.
									//for now it seems to work.

#define lcd_128x32

#endif /* CONFIG_H_ */