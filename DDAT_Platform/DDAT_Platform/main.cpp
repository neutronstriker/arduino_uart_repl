#include "config.h"

#include "Arduino.h"
//#define ARDUINO 101 //this is defined in project properties
//#define F_CPU 16000000UL //it is defined in Project properties

//#include "../../uartlib.h"
#include <util/delay.h>

#include "i2c.h"
#include "SPI.h"

#include "GPIO.h"

#include "myStack.h"

#include "Adafruit_SSD1306.h"

#define OLED_ADD 0x3C

#define F_STR(x) (__FlashStringHelper*)PSTR(x) //to be used only for Arduino print class based functions

Adafruit_SSD1306 lcd;

void loop();
void initPlatform();
void readSerialCmd();
void i2c_internal_pullup(bool state);		//later this should be included in i2c_lib
void uart2Lcd();
void showPinOut();
void stop_streaming();
uint8_t readUntilToken(char* buffer, uint8_t token, uint8_t maxlen);
void usage();

#define CMD_REPLY_BUFFER_LENGTH		64		//initially it was 32, but in order to accommodate i2cwrite instruction, it has been increased to 64.

char btCmdReplyBuffer[CMD_REPLY_BUFFER_LENGTH]={0};
uint8_t uartBufCharCount=0;

//lets try to keep all command keywords within 4 letters, to improve performance by reducing search time,
//however it will be better if we can using strcmp_P and make sure that commands are executed when followed by cr,lf etc. --done
const char PROGMEM cmd1[]="hello";		//later we can move these cmds to code memory and use strstr_P() type functions --done
const char PROGMEM cmd2[]="test";
const char PROGMEM cmd3[]="show";		//show pinout
const char PROGMEM cmd4[]="set";		//set IO pin state
const char PROGMEM cmd5[]="get";		//get IO pin state
const char PROGMEM cmd6[]="setp";		//set input pull up
const char PROGMEM cmd7[]="seti";		//set input
const char PROGMEM cmd8[]="aref";		//set analog reference, arefi:internal, arefx:external, arefv:vcc
const char PROGMEM cmd9[]="aget";		//get analog value
const char PROGMEM cmd10[]="stream";	//'a' analog, 'd' digital data to serial continuously, 'p' for port and 's' for stop.
const char PROGMEM cmd11[]="tog";		//Toggle GPIO Pin
const char PROGMEM cmd12[]="spi";
const char PROGMEM cmd13[]="ate";		//enable terminal echo command.
const char PROGMEM cmd14[] ="help";		//print list of cmds and their usage.
const char PROGMEM cmd15[] ="atde";		//disable terminal echo cmd
const char PROGMEM cmd16[] ="snoope";	//enable Snoop
const char PROGMEM cmd17[] ="snoopd";	//disable snoop
const char PROGMEM cmd18[] ="udis";		//disable UART, only way to enable back is to reset.
const char PROGMEM cmd19[] ="wspr";		//White space remove
const char PROGMEM cmd20[] ="wspk";		//White space keep
const char PROGMEM cmd21[] = "i2cwrite";	//i2c write instruction
const char PROGMEM cmd22[] = "i2cread";	//i2c read instruction
const char PROGMEM cmd23[] = "i2cping";	//i2c ping instruction
const char PROGMEM cmd24[] = "i2cscan";	//scan I2C bus for devices
const char PROGMEM cmd25[] = "case_e";	//enable case sensitivity
const char PROGMEM cmd26[] = "case_d";	//disable case sensitivity
const char PROGMEM cmd27[] = "bspace_e";	//backspace detect
const char PROGMEM cmd28[] = "bspace_d";	//backspace don't detect

const char PROGMEM cmd29[] = "spi_config";	//Configure SPI, CS pin and Clock Freq, and CS ACTIVE LEVEL
const char PROGMEM cmd30[] = "spi_write";	//SPI Write data bytes, Max 8
const char PROGMEM cmd31[] = "spi_csh";		//SPI CS pin High
const char PROGMEM cmd32[] = "spi_csl";		//SPI CS PIN LOW
const char PROGMEM cmd33[] = "spi_read";	//SPI Read data bytes, Max 8
const char PROGMEM cmd34[] = "spi_en";		//enable SPI module
const char PROGMEM cmd35[] = "spi_dis";		//Disable SPI module
const char PROGMEM cmd36[] = "reset";
const char PROGMEM cmd37[] = "pwmv"; //pwmv<pin> <value:0-255>
const char PROGMEM cmd38[] = "pwmf"; //pwmf<pin> <value:[1|8|32|64|128|256|1024]>; set pwm frequency

const char stop=0x03;					//ETX(END of Transmission) or CONTOL^C character can be used to stop streams fast.

/************************************************************************/
/* First we need to design the spi command structure i have two options one is we can use spi in exclusive mode
i.e. once we start spi we will not be able to give any other commands other send receive config etc
but still we will be able to monitor the data streams on LCD.

Or we can use in normal mode, but in this we will have different commands for SPI send, spi receive, spi config 
and then if we use send or receive in can use other commands in between but it will real complex to implement this
especially in slave mode when you are waiting for a data byte(but it can accomplishing using interrupt in slave mode).                                                                     */
/************************************************************************/

//const char PROGMEM cmd11[]="pwm";		//try to set PWM value.

//next we can commands for SPI read,write, I2C read, write, scan, (SoftwareSerial to HardwareSerial Pass through function etc. nope, not this one
//not a good idea at all because there will be lot of delays because once it receives any character it goes through the whole input parsing
//and then is passed to other serialport so it is delayed a lot. any way i don't feel this serial to serial pass through much useful. )


//More importantly I am also going to add SPI slave and I2C slave with custom address configuration and ACK response option
//this might help in emulating or snooping on a particular protocol debugging.

//I should also have a feature to send the data received by UART as response to SPI or I2C so we can emulate.

uint8_t analog_stream_pin;
uint8_t analog_stream_flag=0;

uint8_t digital_stream_pin;
uint8_t digital_stream_flag=0;

uint8_t port_stream=0;
uint8_t case_sensitivity_status=0;

uint8_t backspace_det=0;

struct spi_config{
	uint8_t cs_pin;//=10;
	uint8_t clk_freq_option;//=1;
	uint8_t spiMode;// = SPI_MODE0;
	uint8_t spiDataOrder;// = MSBFIRST;		//1 for MSB first and 0 for LSB first
	uint8_t spiClkDiv; //SPI_CLOCK_DIV4
}spi_config;


struct spi_cmd_structure{
	uint8_t byte_count;
	uint8_t decoded_byte_count;
	uint8_t data[8];
	uint8_t type;
	}spi_cmd_structure;

#define SPI_WRITE_CMD	0
#define SPI_READ_CMD	1
#define SPI_CONFIG_CMD	2
//SPI enable, disable, CS high and CS low don't need parsing

#define FOR_I2C	0
#define FOR_SPI	1

struct i2c_cmd_structure{
	uint8_t device_address;
	uint8_t register_address;
	uint8_t byte_count;
	uint8_t decoded_byte_count;
	uint8_t data[8];
	uint8_t type;
}i2c_cmd_structure;

#define I2C_WRITE_CMD	0
#define I2C_READ_CMD	1
#define I2C_PING_CMD	2
#define I2C_SCAN_CMD	3
uint8_t parse_spi_cmd( char * data, uint8_t cmd_type );
uint8_t parse_i2c_cmd(char * data, uint8_t cmd_type);
int getParam(char * data, uint8_t token, uint8_t type);
uint8_t populateData(char *data, uint8_t max_bytes, uint8_t type, uint8_t i2c_spi);
void watchDog_off();
void resetSysUsingWDT();


uint8_t	last_rst_was_by_wdt=0;		//this flag tells you that if last rst was by WDT, this is controlled in watchDog_off(), however we are not using now.

//uint8_t ate_enabled=0;

uint8_t isPwmPin(uint8_t pin);
uint8_t setPwmFrequency(int pin, int divisor);

int main(void)
{
	watchDog_off();	//this has to be called before global interrupt flag is set.
	
	initPlatform();		//Initialize all platform devices like UART and Display.
	
	delay(2000);
	
	showPinOut();		//show pinout
	
	while(1)
	{
		
		readSerialCmd();	// this will read the data from incoming serial buffer and take action.
		loop();
		if (serialEventRun) serialEventRun();	//Polls the Serial port for new data and updates Buffer accordingly
												// if serialEvent() function is used
	}

	return 0;
}

void loop()
{
	//uart2Lcd();
	
	//for now I am just using simple conditions but if later more than one stream types are enabled than se can
	//make a structured data format with a defined constant order in which we can pack and print the data to uart.
	//likw {0,1023,0x3f}, digital, analog, port
	if(analog_stream_flag || digital_stream_flag || port_stream)
	{
		lcd.clearDisplay();
		lcd.setCursor(0,0);
		lcd.setTextColor(WHITE);
		lcd.setTextSize(2);
	
			if (digital_stream_flag)
			{
				uint8_t data = digitalRead(digital_stream_pin);
			
				Serial.print(data);
				
				if(analog_stream_flag || port_stream)
					Serial.print(',');
				else
					Serial.print("\r\n");
				
				
				lcd.setCursor(0,0);
				
				lcd.print('D');
				lcd.print(digital_stream_pin);
				
				lcd.setCursor(0,16);
				
				lcd.print(data);
			}
			
			if(analog_stream_flag)
			{
				uint16_t data = analogRead(analog_stream_pin);
				
				Serial.print(data);
				
				if (port_stream)
					Serial.print(',');
				else
					Serial.print("\r\n");
				
				
				lcd.setCursor(48,0);
				
				lcd.print('A');
				lcd.print(analog_stream_pin-A0);
				
				lcd.setCursor(36,16);
				lcd.print(data);
				
			}
			
			if(port_stream == 1)
			{
				Serial.println(PINB,HEX);
				
				lcd.setCursor(96,0);
				
				lcd.print("PB");
				
				lcd.setCursor(96,16);
				lcd.print(PINB,HEX);
			}
			
			else if(port_stream == 2)
			{
				Serial.println(PINC,HEX);
				
				lcd.setCursor(96,0);
				
				lcd.print("PC");
				
				lcd.setCursor(96,16);
				lcd.print(PINC,HEX);
			}
			
			else if (port_stream == 3)
			{
				Serial.println(PIND,HEX);
				
				lcd.setCursor(96,0);
				
				lcd.print("PD");
				
				lcd.setCursor(96,16);
				lcd.print(PIND,HEX);
			}	
	
	
		lcd.display();
	}
	
}


void initPlatform()
{
	init();	//Initialize Arduino Core
	
	Serial.begin(57600);//increases a lot or Ram and Rom usage disable after diagnostics and recompile.
	
	Serial.print(F_STR("DDAT_INITIALISED\r\n"));
	
	//i2c_init(100000);	//it is called with 400Khz by adafruit display lib so not necessary any more
	
	i2c_internal_pullup(true);	//this is not required since pullups are present in LCD board, however
									//if we add more devices to bus we should add external pull-ups.
									//I also found an interesting thing which is that I2C bus doesn't hang because of my code
									//if there are pullups present. Previously I used to think if there are no devices and I
									//try to write to a device then it will hang the bus but actually my code is no that bad.
									//(I still need to check on the timeout issue though). If pullups are present the it works
									//out fine most of the time.
									
	lcd.begin(SSD1306_SWITCHCAPVCC, OLED_ADD);
	
	lcd.dim(true);//dim the display brightness
	
	lcd.clearDisplay();
	lcd.setCursor(0,0);
	lcd.setTextColor(WHITE);
	lcd.setTextWrap(true);
	lcd.setTextSize(2);
	lcd.println(F_STR("    DDAT"));
	lcd.println(F_STR("  PLATFORM"));
	lcd.display();

}

void i2c_internal_pullup(bool state)
{
	GPIO sda(A4);
	GPIO scl(A5);
	
	if(state == true)
	{
		sda.setInputPullUp();
		scl.setInputPullUp();	
	}
	else
	{
		sda.setInputPullDown();
		sda.setInputPullDown();
	}
}

void reset_millis()
{
	
	extern volatile unsigned long timer0_overflow_count;
	extern volatile unsigned long timer0_millis;
	timer0_overflow_count=0;
	timer0_millis=0;
	
}

uint16_t minutes()
{
	return (millis()/1000)/60; //check this that division operation can seriously decrease performance
                            //because there is not hardware division algorithm
                            //try to take minutes and seconds as global values and run only seconds that will update
                           //the minutes value also once in every 60 seconds instead of once on every call.
}

uint8_t seconds()
{
	return (millis()/1000)%60; //check this that modulo operation can seriously decrease performance
}                               //find an optimized way try a mix of >> and subtraction instead


ISR(INT1_vect)
{
}

ISR(INT0_vect)
{
}


void replyOK()
{
	Serial.println("OK");
}

void uart2Lcd()
{
	//this function and readSerialCmd() can't co-exist together since in readSerialCmd() it reads bytes from serialbuffer
	//that means data is clear so sometimes typed data will not reach lcd.
	static int count =0;
	while(!Serial.available());
	char c = Serial.read();
	lcd.print(c);
	lcd.display();
	count++;
	if(count==20)
	{
		lcd.clearDisplay();
		lcd.setCursor(0,0);
		count=0;
	}
	
}

void readSerialCmd()
{
		//char *pos;
		char numeric[7];
		uint8_t token_char = '\r';
		uint8_t num;
		
		/*
			Don't Provide both CR and LF after cmds provide only CR. LF is not honoured and is taken up as a character thus, if anywhere
			whole word match is required that condition will fail.
		*/

#ifdef SERIAL_CR_DETECT
	if(Serial.crStatusDetected())	//experimental feature.
#endif
		if(Serial.available())
		{
			#ifdef PRINT_DEBUG_MSG
				Serial.println(F_STR("data available"));
			#endif
			
				
				uartBufCharCount = readUntilToken(btCmdReplyBuffer,token_char,CMD_REPLY_BUFFER_LENGTH);	
				
				if (case_sensitivity_status)
				{
					strlwr(btCmdReplyBuffer); //convert string to lower case
				}
				
								
				#ifdef PRINT_DEBUG_MSG
					Serial.print("Char Count ");
					Serial.println(uartBufCharCount);
				#endif
				
				if(uartBufCharCount == 0)
				{
					#ifdef PRINT_DEBUG_MSG
						Serial.println("No Character found");
					#endif
					
					return;				//i.e. the '\r' was not found in string/array
				}
				
				/*
				//Read until return is detected or 12 bytes from serial Fifo to buffer.
				uartBufCharCount = Serial.readBytes(btCmdReplyBuffer,12);
				
				
				
				//terminate the string at the point where we find the token character which is '\r' <CR> in this case.
				uint8_t char_pos;
				for(char_pos=0;char_pos<uartBufCharCount;char_pos++)
				{
					if(btCmdReplyBuffer[char_pos]==token_char)
					{
						btCmdReplyBuffer[char_pos]='\0';		//terminate the string at that point so that unnecessary string processing
						break;									//can be avoided.
					}
					
				}
				
				//when 'char_pos' value is same as total character count that means it never found the character,
				//otherwise a break instruction would have been executed. 
				if(char_pos==uartBufCharCount)
				{
					#ifdef PRINT_DEBUG_MSG
						Serial.println("No Carriage return detected");
					#endif
					return;				//i.e. the '\r' was not found in string/array
				}
				*/
				
			#ifdef PRINT_DEBUG_MSG
				uint8_t i=0;
				while(i<=uartBufCharCount)
				{
					Serial.print(btCmdReplyBuffer[i], HEX);
					Serial.print(' ');
					i++;
				}
				//serial debug messages here
			#endif // PRINT_DEBUG_MSG			
			
			
			if (strncmp_P(btCmdReplyBuffer,cmd4,3)==0)
			{
				
				if(btCmdReplyBuffer[4] < 0x30 || btCmdReplyBuffer[4] > 0x39)
				{
					Serial.println(F_STR("ERROR_1"));
					return;
				}
				//strncpy(numeric,pos+4,2);seth
				strncpy(numeric,btCmdReplyBuffer+4,7);		//Now we should be able to check even if someone types seth002 instead of seth2
				//numeric[2]='\0';
				//here we need to implement the check for not exceeding pin numbers available
				num=atoi(numeric);
				if(num > A5)
				{
					Serial.println(F_STR("ERROR_2"));
					return;
				}
				#ifdef PRINT_DEBUG_MSG
				Serial.print(F_STR("pin num is "));
				Serial.println(num);
				#endif
				
				GPIO io(num);		//instantiate GPIO object.
				
				lcd.clearDisplay();
				lcd.setCursor(0,0);
				lcd.setTextSize(2);
				
				lcd.print(F_STR("GPIO "));
				lcd.println(num);
				lcd.print(F_STR("SET "));

				
				switch(btCmdReplyBuffer[3])
				{
					
					case 'h':
					io.High();
					lcd.print(F_STR("HIGH"));
					break;
					
					case 'l':
					io.Low();
					lcd.print(F_STR("LOW"));
					break;
					
					case 'p':
					io.setInputPullUp();
					lcd.print(F_STR("P_UP"));
					break;
					
					case 'i':
					io.setInputPullDown();
					lcd.print(F_STR("INPUT"));
					break;
					
					default:
					lcd.print(F_STR("CMD_ER"));
					Serial.println(F_STR("ERROR3"));
					break;
				}
				
				lcd.display();
				
				replyOK();
			}
			
			else if (strncmp_P(btCmdReplyBuffer,cmd11,3)==0)
			{
				if(btCmdReplyBuffer[3] < 0x30 || btCmdReplyBuffer[3] > 0x39)
				{
					Serial.println(F_STR("ERROR_1"));
					return;
				}
				
				strncpy(numeric,btCmdReplyBuffer+3,7);		//Now we should be able to check even if someone types tog002 instead of tog2
				
				//here we need to implement the check for not exceeding pin numbers available
				num=atoi(numeric);
				if(num > A5)
				{
					Serial.println(F_STR("ERROR_2"));
					return;
				}
				
				#ifdef PRINT_DEBUG_MSG
				Serial.print(F_STR("pin num is "));
				Serial.println(num);
				#endif
				
				GPIO io(num);		//instantiate GPIO object.
				
				io.toggle();		//toggle the pin.
				
				lcd.clearDisplay();
				lcd.setCursor(0,0);
				lcd.setTextSize(2);
				
				lcd.print(F_STR("GPIO "));
				lcd.println(num);
				lcd.print(F_STR("TOGGLED "));
				lcd.print(digitalRead(num));
				lcd.display();
				
				Serial.println(digitalRead(num));
				
				replyOK();
			}
			
			else if (strncmp_P(btCmdReplyBuffer,cmd5,3)==0)
			{
				
				
				if(btCmdReplyBuffer[3] < 0x30 || btCmdReplyBuffer[3] > 0x39)
				{
					Serial.println(F_STR("ERROR_1"));
					return;
				}
				
				strncpy(numeric,btCmdReplyBuffer+3,7);		//Now we should be able to check even if someone types get002 instead of get2
				
				//here we need to implement the check for not exceeding pin numbers available
				num=atoi(numeric);
				if(num > A5)
				{
					Serial.println(F_STR("ERROR_2"));
					return;
				}
				
				#ifdef PRINT_DEBUG_MSG
				Serial.print(F_STR("pin num is "));
				Serial.println(num);
				#endif
				
				GPIO io(num);		//instantiate GPIO object.
				
				
				lcd.clearDisplay();
				lcd.setCursor(0,0);
				lcd.setTextSize(2);
				
				lcd.print(F_STR("GPIO "));
				lcd.println(num);
				lcd.print(F_STR("STATE "));
				
				if(io.getState())
				{
					lcd.print(F_STR("HIGH"));
					Serial.println('1');
				}
				
				else
				{
					lcd.print(F_STR("LOW"));
					Serial.println('0');
				}
				
				lcd.display();
				
				replyOK();
			}
			
			else if(strncmp_P(btCmdReplyBuffer,cmd9,4)==0)
			{
				
				if(btCmdReplyBuffer[4] < 0x30 || btCmdReplyBuffer[4] > 0x39)
				{
					Serial.println(F_STR("ERROR_1"));
					return;
				}
				
				strncpy(numeric,btCmdReplyBuffer+4,7);		//Now we should be able to check even if someone types aget002 instead of aget2
				
				//here we need to implement the check for not exceeding pin numbers available
				num=atoi(numeric);
				if(num > 7)			//we will use 0 for A0 and 7 for A7
				{
					Serial.println(F_STR("ERROR_2"));
					return;
				}
				
				#ifdef PRINT_DEBUG_MSG
				Serial.print(F_STR("pin num is "));
				Serial.println(num);
				#endif
				
				lcd.clearDisplay();
				lcd.setCursor(0,0);
				lcd.setTextSize(2);
				
				lcd.print(F_STR("ANALOG "));
				lcd.println(num);
				
				uint16_t val = analogRead(A0+num);
				
				lcd.print(val);
				Serial.println(val);
				
				lcd.display();
				
				replyOK();
			}
			
			//pwm control
			else if(strncmp_P(btCmdReplyBuffer,cmd37,4)==0)
			{
				if(btCmdReplyBuffer[4] < 0x30 || btCmdReplyBuffer[4] > 0x39)
				{
					Serial.println(F_STR("ERROR_1"));
					return;
				}
				
				strncpy(numeric,btCmdReplyBuffer+4,2);		
				
				//here we need to implement the check for not exceeding pin numbers available
				num=atoi(numeric);
				
				#ifdef PRINT_DEBUG_MSG
				Serial.print(F_STR("pin num is "));
				Serial.println(num);
				#endif
				
				if(!isPwmPin(num))			
				{
					Serial.println(F_STR("ERROR_2"));
					return;
				}
				
				
				uint8_t pwmVal = 0;
				strncpy(numeric,btCmdReplyBuffer+4+2,uartBufCharCount-(4+2));
				
				pwmVal=atoi(numeric);
				
				#ifdef PRINT_DEBUG_MSG
				Serial.print(F_STR("pwm val is "));
				Serial.println(pwmVal);
				#endif
				
				if (pwmVal > 255)
				{
					Serial.println(F_STR("ERROR_3"));
					return;
				}
				
				analogWrite(num,pwmVal);
								
				lcd.clearDisplay();
				lcd.setCursor(0,0);
				lcd.setTextSize(2);
				
				lcd.print(F_STR("PWM "));
				lcd.println(num);
				
				lcd.print(pwmVal);
				
				
				lcd.display();
				replyOK();
			}
			
			//pwm frequency divisor setting
			else if(strncmp_P(btCmdReplyBuffer,cmd38,4)==0)
			{
				if(btCmdReplyBuffer[4] < 0x30 || btCmdReplyBuffer[4] > 0x39)
				{
					Serial.println(F_STR("ERROR_1"));
					return;
				}
				
				strncpy(numeric,btCmdReplyBuffer+4,2);
				
				//here we need to implement the check for not exceeding pin numbers available
				num=atoi(numeric);
				
				#ifdef PRINT_DEBUG_MSG
				Serial.print(F_STR("pin num is "));
				Serial.println(num);
				#endif
				
				if(!isPwmPin(num))
				{
					Serial.println(F_STR("ERROR_2"));
					return;
				}
				
				
				uint16_t pwmFeqDiv = 0;
				strncpy(numeric,btCmdReplyBuffer+4+2,uartBufCharCount-(4+2));
				
				pwmFeqDiv=atoi(numeric);
				
				#ifdef PRINT_DEBUG_MSG
				Serial.print(F_STR("pwm freq divisor is "));
				Serial.println(pwmFeqDiv);
				#endif
				
				if (setPwmFrequency(num,pwmFeqDiv)==0)
				{
					Serial.println(F_STR("ERROR_3"));
					return;
				}
								
				lcd.clearDisplay();
				lcd.setCursor(0,0);
				lcd.setTextSize(2);
				
				lcd.print(F_STR("PWMF "));
				lcd.println(num);
				
				lcd.print(pwmFeqDiv);
				
				
				lcd.display();
				replyOK();
			}
			
			else if(strncmp_P(btCmdReplyBuffer,cmd8,4)==0)
			{
				if(btCmdReplyBuffer[5]!='\0')
				{
					Serial.println(F_STR("ERROR_1"));
					return;
				}
				
				lcd.clearDisplay();
				lcd.setCursor(0,0);
				lcd.setTextSize(2);
				lcd.print(F_STR("ANALOG REF"));		//this has already 10 chars so if we give println then it goes one more line below
				//and chars are invisible.
				switch(btCmdReplyBuffer[4])
				{
					case 'i':
					analogReference(INTERNAL);
					lcd.print(F_STR("INTERNAL"));
					break;
					
					case 'x':
					analogReference(INTERNAL);
					lcd.print(F_STR("EXTERNAL"));
					break;
					
					case 'v':
					analogReference(DEFAULT);
					lcd.print(F_STR("VCC"));
					break;
					
					default:
					Serial.println(F_STR("ERROR_2"));
					break;
				}
				
				lcd.display();
				
				replyOK();
			}
			
			else if(strncmp_P(btCmdReplyBuffer,cmd10,5)==0)
			{
				
				
				switch(btCmdReplyBuffer[6])
				{
					case 'a':
						if(btCmdReplyBuffer[7] < 0x30 || btCmdReplyBuffer[7] > 0x39)
						{
							Serial.println(F_STR("ERROR_1"));
							return;
						}
						
						strncpy(numeric,btCmdReplyBuffer+7,7);
						
						//here we need to implement the check for not exceeding pin numbers available
						num=atoi(numeric);
						if(num > 7)			//we will use 0 for A0 and 7 for A7
						{
							Serial.println(F_STR("ERROR_2"));
							return;
						}
						
						analog_stream_pin = A0+num;
						analog_stream_flag = 1;
						
					break;
					
					case 'd':
						if(btCmdReplyBuffer[7] < 0x30 || btCmdReplyBuffer[7] > 0x39)
						{
							Serial.println(F_STR("ERROR_1"));
							return;
						}
						
						strncpy(numeric,btCmdReplyBuffer+7,7);		//Now we should be able to check even if someone types get002 instead of get2
						
						//here we need to implement the check for not exceeding pin numbers available
						num=atoi(numeric);
						if(num > A5)
						{
							Serial.println(F_STR("ERROR_2"));
							return;
						}
						
						digital_stream_pin=num;
						digital_stream_flag=1;
						
					break;
					
					case 'p':
						if(btCmdReplyBuffer[7]=='b')
						{
							port_stream=1;
						}
						else if (btCmdReplyBuffer[7]=='c')
						{
							port_stream=2;
						}
						else if(btCmdReplyBuffer[7]=='d')
						{
							port_stream=3;
						}
						else
						{
							port_stream = 0;
							Serial.println(F_STR("ERROR_4"));
						}
					break;
					
					case 's':		//stop all streaming.
						stop_streaming();
					break;
					
					default:Serial.println(F_STR("ERROR_3"));
					break;
				}
				
				replyOK();
				
			}
			
			else if(strcmp_P(btCmdReplyBuffer,cmd1)==0)	//search for Token 1
			{	
			
				//message here	
						
				replyOK();		//command processed response
								//if OK is not received that means CMD not processed by
				lcd.clearDisplay();
				lcd.setCursor(0,0);
				lcd.setTextSize(2);
				lcd.print(F_STR("hello"));
				lcd.dim(false);
				lcd.display();
				//action here
			
			}
			
			else if(strcmp_P(btCmdReplyBuffer,cmd2)==0)	//search for Token 2
			{
				
				//message
				
				replyOK();
				
				lcd.clearDisplay();
				lcd.setCursor(0,0);
				lcd.setTextSize(2);
				lcd.print(F_STR("Test Success"));
				lcd.dim(true);
				lcd.display();
				//action
			}
			
			else if(strcmp_P(btCmdReplyBuffer,cmd3)==0)	//search for Token 3
			{
				
				//message	
				replyOK();
				showPinOut();
				//action
			}
			
			else if(strcmp_P(btCmdReplyBuffer,cmd13)==0)	//experimental feature
			{
				Serial.enableEcho();
				//ate_enabled=1;
				Serial.println("ECHO_ENABLED");
				replyOK();
			}
			
			else if(strcmp_P(btCmdReplyBuffer,cmd15)==0)	//experimental feature
			{
				Serial.disableEcho();
				Serial.println("ECHO_DISABLED");
				replyOK();
			}
			
			else if(strncmp_P(btCmdReplyBuffer,cmd16,5)==0)	//experimental feature
			{
				if(btCmdReplyBuffer[5]=='e')
				{
					Serial.snoopModeEnable();
				}
				else if (btCmdReplyBuffer[5]=='d')
				{
					Serial.snoopModeDisable();
				}
				else
				{
					Serial.println("ERROR_1");
				}
				replyOK();
			}
			
			else if(strcmp_P(btCmdReplyBuffer,cmd18)==0)	//experimental feature
			{
				replyOK();
				Serial.end();
			}
			
			else if(strncmp_P(btCmdReplyBuffer,cmd19,3)==0)	//experimental feature
			{
				if(btCmdReplyBuffer[3]=='r')
				{
					Serial.exWhiteSpaces();
					Serial.println(F_STR("EXCLUDING WHITE SPACE"));
				}
				else if (btCmdReplyBuffer[3]=='k')
				{
					Serial.inWhiteSpaces();
					Serial.println(F_STR("INCLUDING WHITE SPACE"));
				}
				else
				{
					Serial.println("ERROR_1");
				}
				replyOK();
			}
			
			else if(strncmp_P(btCmdReplyBuffer,cmd21,3)==0)	//experimental feature
			{
				if(strncmp_P(btCmdReplyBuffer,cmd21,8)==0)
				{
					if(!parse_i2c_cmd(btCmdReplyBuffer,I2C_WRITE_CMD))
					{
						Serial.println(F_STR("ERROR_2"));
						return;
					}
					
					if(i2c_cmd_structure.decoded_byte_count != i2c_cmd_structure.byte_count)
					{
						Serial.println(F_STR("ERROR! Byte Count doesn't match number of bytes"));
						
						#ifndef	PRINT_DEBUG_MSG 
							return;
						#endif
					}
					if (!i2c_ping(i2c_cmd_structure.device_address))
					{
						Serial.println(F_STR("ERROR! Device Not Found/Respoding"));
						return;
					}
					uint8_t status = i2c_write(i2c_cmd_structure.device_address,i2c_cmd_structure.register_address,i2c_cmd_structure.data,i2c_cmd_structure.decoded_byte_count);
					if(status)
					{
						Serial.print(F_STR("Status: "));
						Serial.println(status);
						Serial.println(F_STR("Data Written Successfully"));
					}
				}
				else if(strncmp_P(btCmdReplyBuffer,cmd22,7)==0)
				{
					if(!parse_i2c_cmd(btCmdReplyBuffer,I2C_READ_CMD))
					{
						Serial.println(F_STR("ERROR_2"));
						return;
					}
					
					if (i2c_cmd_structure.byte_count > 8 || i2c_cmd_structure.byte_count == 0)
					{
						Serial.println(F_STR("ERROR! Cannot read more than 8 bytes in one shot."));
						
						#ifndef	PRINT_DEBUG_MSG
							return;
						#endif
					}
					if (!i2c_ping(i2c_cmd_structure.device_address))
					{
						Serial.println(F_STR("ERROR! Device Not Found/Respoding"));
						return;
					}
					
					uint8_t status = i2c_read(i2c_cmd_structure.device_address,i2c_cmd_structure.register_address,i2c_cmd_structure.data,i2c_cmd_structure.byte_count);
					
					if (status)
					{
						Serial.print(F_STR("Status: "));
						Serial.println(status);
						//Serial.print(F_STR("Data Read:"));
						Serial.print(F_STR("Data Read:\r\n"));
						for(uint8_t i=0;i<i2c_cmd_structure.byte_count;i++)
						{
							Serial.print(i2c_cmd_structure.data[i],i2c_cmd_structure.type);
							//Serial.print(' ');
							if(i<i2c_cmd_structure.byte_count-1)
								Serial.print(',');
							else Serial.print("\r\n");
						}
					}
					else
					{
						Serial.println(F_STR("ERROR_3"));
						return;
					}
					
					
				}
				
				else if(strncmp_P(btCmdReplyBuffer,cmd23,7)==0)
				{
					if(!parse_i2c_cmd(btCmdReplyBuffer,I2C_PING_CMD))
					{
						Serial.println(F_STR("ERROR_2"));
						return;
					}
					
					if(i2c_ping(i2c_cmd_structure.device_address))
					{
						Serial.println(F_STR("Device Responded to Ping"));
					}
					else
					{
						Serial.println(F_STR("Device Not Responding to Ping"));
					}
					
				}
				else if (strncmp_P(btCmdReplyBuffer,cmd24,7)==0)
				{
					Serial.println(F_STR("Scanning I2C bus..."));
					Serial.print(F_STR("Device Address:"));
					
					uint8_t count = 0;
					for (uint8_t i=1; i< 128; i++)
					{
						if (i2c_ping(i))
						{
							Serial.print(i,HEX);
							Serial.print(' ');
							count++;
						}
						
					}
					
					Serial.println(F_STR("\r\nScan Complete."));
					Serial.print(count);
					Serial.println(F_STR(" Devices Found."));
						
				}
				else
				{
					Serial.println("ERROR_1");
				}
				replyOK();
			}
			
			else if (strncmp_P(btCmdReplyBuffer,cmd25,4)==0)
			{
				if (strcmp_P(btCmdReplyBuffer,cmd25)==0)
				{
					case_sensitivity_status = 0;
					Serial.println(F_STR("Case Sensitivity Enabled"));
				}
				else if (strcmp_P(btCmdReplyBuffer,cmd26)==0)
				{
					case_sensitivity_status = 1;
					Serial.println(F_STR("Case Sensitivity Disabled"));
				}
				else
				{
					Serial.println(F_STR("ERROR_1"));
				}
				replyOK();
			}
			
			else if (strncmp_P(btCmdReplyBuffer,cmd27,7)==0)
			{
				if (strcmp_P(btCmdReplyBuffer,cmd27)==0)
				{
					backspace_det = 1;
					Serial.println(F_STR("Backspace detection Enabled"));
				}
				else if (strcmp_P(btCmdReplyBuffer,cmd28)==0)
				{
					backspace_det = 0;
					Serial.println(F_STR("Backspace detection Disabled"));
				}
				else
				{
					Serial.println(F_STR("ERROR_1"));
				}
				replyOK();
			}
			
			
			else if (strncmp_P(btCmdReplyBuffer,cmd34,4)==0)
			{
				if (strcmp_P(btCmdReplyBuffer,cmd34)==0)
				{
					SPI.begin();
					Serial.println(F_STR("SPI Enabled"));
				}
				else if (strcmp_P(btCmdReplyBuffer,cmd35)==0)
				{
					SPI.end();
					Serial.println(F_STR("SPI Disabled"));
				}
				else if (strncmp_P(btCmdReplyBuffer,cmd29,10)==0)
				{
					if (parse_spi_cmd(btCmdReplyBuffer,SPI_CONFIG_CMD))
					{
						SPI.setBitOrder(spi_config.spiDataOrder);
						SPI.setClockDivider(spi_config.spiClkDiv);
						SPI.setDataMode(spi_config.spiMode);
					}
					else
					{
						Serial.println(F_STR("ERROR_2"));
						return;
					}
					
					
				}
				else if (strncmp_P(btCmdReplyBuffer,cmd30,9)==0)
				{
					if (SPI.isSPIenabled()==0)
					{
						Serial.println(F_STR("ERROR! Enable SPI first"));
						return;
					}
					
					if(parse_spi_cmd(btCmdReplyBuffer,SPI_WRITE_CMD))
					{
						if (spi_cmd_structure.byte_count != spi_cmd_structure.decoded_byte_count)
						{
							Serial.println(F_STR("ERROR! Byte Count doesn't match number of bytes"));
							
							#ifndef	PRINT_DEBUG_MSG
								return;
							#endif
						}
						
						if (spi_cmd_structure.byte_count > 8 || spi_cmd_structure.byte_count == 0)
						{
							Serial.println(F_STR("ERROR! Cannot Process more than 8 bytes in one shot."));
							
							#ifndef	PRINT_DEBUG_MSG
								return;
							#endif
						}
						
						for(uint8_t i=0;i<spi_cmd_structure.byte_count;i++)
						{
							SPI.transfer(spi_cmd_structure.data[i]);
						}
						Serial.println(F_STR("Data Sent"));
						
					}
					else
					{
						Serial.println(F_STR("ERROR_2"));
						return;					
					}
			
				}
				else if (strncmp_P(btCmdReplyBuffer,cmd33,8)==0)
				{
					if (SPI.isSPIenabled()==0)
					{
						Serial.println(F_STR("ERROR! Enable SPI first"));
						return;
					}
					
					
					if(parse_spi_cmd(btCmdReplyBuffer,SPI_READ_CMD))
					{
						if (spi_cmd_structure.byte_count > 8 || spi_cmd_structure.byte_count == 0)
						{
							Serial.println(F_STR("ERROR! Cannot Process more than 8 bytes in one shot."));
							
							#ifndef	PRINT_DEBUG_MSG
								return;
							#endif
						}
						
						
						Serial.print(F_STR("Data Read:\r\n"));
						for(uint8_t i=0;i<spi_cmd_structure.byte_count;i++)
						{
							spi_cmd_structure.data[i] = SPI.transfer(0xff);//read data
							Serial.print(spi_cmd_structure.data[i],spi_cmd_structure.type);
							if(i<spi_cmd_structure.byte_count-1)
							Serial.print(',');
							else Serial.print("\r\n");
						}
										
					}
					else
					{
						Serial.println(F_STR("ERROR_2"));
						return;

					}
					
				}
				else if (strcmp_P(btCmdReplyBuffer,cmd31)==0)
				{
					GPIO CS(spi_config.cs_pin);
					CS.High();
				}
				else if (strcmp_P(btCmdReplyBuffer,cmd32)==0)
				{
					GPIO CS(spi_config.cs_pin);
					CS.Low();
				}
				
				else
				{
					Serial.println(F_STR("ERROR_1"));
				}
				replyOK();
			}
			
			else if (strcmp_P(btCmdReplyBuffer,cmd36)==0)
			{
				replyOK();
				resetSysUsingWDT(); //reset device using WDT.
			}
			
			else if(btCmdReplyBuffer[0]==stop)
			{
				stop_streaming();
				//you have to also press enter after control-c other wise command will not be registered.
				//or you have to move this to the top most even before CR detection.
				//no reply because it is a control-c.
			}
			
			else if(strcmp_P(btCmdReplyBuffer,cmd14)==0)	//experimental feature
			{
				usage();
				replyOK();
			}
			
			else
			{
				Serial.println(F_STR("ERROR(!)"));	
			}
									
		}
		
}

uint8_t isPwmPin(uint8_t pin)
{
	if (pin == 3 || pin == 5 || pin == 6 || pin == 9 || pin == 10 || pin == 11)
		return 1;
	else
		return 0;
	
}

void watchDog_off()
{
	/* make sure this function is called before global interrupt (Sei) is enabled
	or first disable it and then call it. */
	
	asm volatile("wdr"::);				//reset watchdog
	if(MCUSR & (1<<WDRF))				//check write this to flag before clearing WDRF flag, so we can check the state by using this.
		last_rst_was_by_wdt = 1;
	MCUSR &= ~(1<<WDRF);				//clear WDRF flag, not sure why we need to do this.
	WDTCSR |= (1<<WDCE) | (1<<WDE);		//following WDTCSR change sequence
	WDTCSR = (1<<WDIF);					//clear the Interrupt flag if it exists because of previous WDT reset,
										//WDE cleared, i.e. WDT is disabled, WDIE is also disabled i.e. No interrupt on WDT expire, 
										//clock timeout period selected to 16ms (minimum Period).
}

void resetSysUsingWDT()
{
	cli();								//clear global interrupt first.
	WDTCSR |= (1<<WDCE) | (1<<WDE);		//following WDTCSR change sequence
	WDTCSR = (1<<WDIF) | (1<<WDE);		//clear the Interrupt flag if it exists because of previous WDT reset,
										//WDE set, i.e. WDT is disabled, WDIE is also disabled i.e. No interrupt on WDT expire,
										//clock timeout period selected to 16ms (minimum Period).
	_delay_ms(50);						//I am not using delay() arduino function because we have disabled interrupt delay will
										//not work properly.
}

void usage()
{
	Serial.print(F_STR("DDAT Platform \r\nUsage:\r\n"));
	Serial.print(F_STR("	sethX: Set X pin High\r\n"));
	Serial.print(F_STR("	setlX: Set X pin Low\r\n"));
	Serial.print(F_STR("	setiX: Set X pin input\r\n"));
	Serial.print(F_STR("	setpX: Set X pin input Pull-up\r\n"));
	Serial.print(F_STR("	getX: get X pin digital Value\r\n"));
	Serial.print(F_STR("	pwmvX Y: set pwm on X pin of Value Y(0-255)\r\n"));
	Serial.print(F_STR("	pwmfX Y: set pwm frequency divisor on X pin of Value Y[1|8|32|64|128|256|1024]\r\n"));
	Serial.print(F_STR("	agetX: get X pin analog Value\r\n"));
	Serial.print(F_STR("	arefX: set analog reference, arefi:internal, arefx:external, arefv:vcc\r\n"));
	Serial.print(F_STR("	togX: toggle X pin digital output\r\n"));
	Serial.print(F_STR("	streamDX: continuously Stream the state of digital Pin X\r\n"));
	Serial.print(F_STR("	streamAX: continuously Stream the data of analog Pin X\r\n"));
	Serial.print(F_STR("	streamPX: continuously Stream the Value of PORT X, where X can be B, C or D in HEX\r\n"));
	Serial.print(F_STR("	streamS: Stop all streams\r\n"));
	Serial.print(F_STR("	i2cscan: Scan I2C bus for all devices\r\n"));
	Serial.print(F_STR("	i2cping: Checks if a requested I2C slave device is responsive or not\r\n"));
	Serial.print(F_STR("	          format: i2cping -s<slave_addr>\r\n"));
	Serial.print(F_STR("	i2cwrite: Write data to requested I2C slave device\r\n"));
	Serial.print(F_STR("	          format:i2cwrite -s<slave_addr> -n<no. of bytes> -r<register_addr> -h<databyte1,databyte2..> [-tHEX:DEC]\r\n"));
	Serial.print(F_STR("	          the sequence of parameters is not important, max number of bytes can be 8, default is HEX type\r\n"));
	Serial.print(F_STR("	          you can specify DEC if you want.\r\n"));
	Serial.print(F_STR("	i2cread: Read data from requested I2C slave device\r\n"));
	Serial.print(F_STR("	          format:i2cread -s<slave_addr> -n<no. of bytes> -r<register_addr> [-tHEX:DEC]\r\n"));
	Serial.print(F_STR("	          the sequence of parameters is not important, max number of bytes can be 8, default is HEX type\r\n"));
	Serial.print(F_STR("	          you can specify DEC if you want.\r\n"));
	Serial.print(F_STR("	show: Show pin-out on OLED Display\r\n"));
	Serial.print(F_STR("	ate: enable Terminal ECHO\r\n"));
	Serial.print(F_STR("	atde: disable Terminal ECHO\r\n"));
	Serial.print(F_STR("	bspace_e: enable Backspace detection\r\n"));
	Serial.print(F_STR("	bspace_d: disable Backspace detection\r\n"));
	Serial.print(F_STR("	case_e: enable Case sensitivity\r\n"));
	Serial.print(F_STR("	case_d: disable Case sensitivity\r\n"));
	Serial.print(F_STR("	wspr: discard white space characters\r\n"));
	Serial.print(F_STR("	wspk: keep white space characters\r\n"));
	Serial.print(F_STR("	reset: resets the device\r\n"));
	Serial.print(F_STR("	snoope: disables TX of AVR and keeps the RX enabled so that we can read data of another UART device connected\r\n"));
	Serial.print(F_STR("	snoopd: Enables the TX of AVR back ON\r\n"));
	Serial.print(F_STR("	udis: disable Uart connection; can be use to make a pass through connection between Bluetooth and another UART device.\r\n"));
	Serial.print(F_STR("	help: Show this help message\r\n"));
	Serial.print(F_STR("\r\nNote: 1.CTRL+C followed by RETURN also stops stream\r\n"));
	Serial.print(F_STR("      2.All three types of stream can be active together and data will be in Comma Separated values\r\n"));
	Serial.print(F_STR("      3.Commands are case sensitive, we need to check this\r\n"));
	Serial.print(F_STR("      4.New Commands have been added please update this list\r\n"));
	
}


void showPinOut()
{
	lcd.clearDisplay();
	lcd.setCursor(0,0);
	lcd.setTextSize(1);
	lcd.println(F_STR("12,11...3,2,G,R,RX,TX"));
	lcd.println(F_STR("13,3.3,AREF,A0-A7"));
	lcd.print(F_STR("           5V,R,G,RAW"));
	lcd.display();
}

void stop_streaming()
{
	port_stream = 0;
	digital_stream_flag=0;
	analog_stream_flag=0;
	lcd.clearDisplay();
	lcd.setCursor(0,0);
	lcd.setTextSize(2);
	lcd.setTextColor(WHITE);
	lcd.println(F_STR("STREAMING"));
	lcd.print(F_STR(" STOPPED"));
	lcd.display();
	Serial.println(F_STR("STREAMING_STOPPED"));
}

uint8_t readUntilToken(char* buffer, uint8_t token, uint8_t maxlen)
{
	/*	The Serial.readBytesUntil() doesn't work that well for me because it does a signed character check, which sometimes
		could be a problem for me because it doesn't work well sometime, especially when I tell it read looking for CR it goes mad
		and returns random characters.
		
		So i decided to do my own version.
	*/
	
	/*
		Another most important thing is that we should not use a mixture of both my uart_function and Serial class
		it creates lot of problems and wastes lot of time to debug, use only one of them in whole program.
		
		One more thing is the Serial.readUntil thing also filled the buffer with the token which was the problem
		some places I have used code which looks to match the whole word in that case it fails.
		
		I used to get only half the codes present in buffer like for hello i got only 6C 6F 0D 00 because 
		the hexbyte used uart_write() internally which creates problem since it uses blocking mode not interrupt
		mode like Serial class and interrupt is already enabled for UDRE so it messes things up. So don't use it
		here anymore.
		
		But the Serial.readBytesUntil is advanced in that it can wait for until we actually send the character 
		to the uart and the max wait period can be set to timeout value. So it can actually let you get all
		the characters into a buffer which we specify until the token is detected, and buffer can be bigger than
		even serialreceive buffer of 64bytes.
		
		But since we are doing parsing only after detecting CR so we don't need this feature, not useful for me
		right now. May be we can use this if required for exclusive SPI and I2C mode to specify data strings.
		
		But my code in this function is slightly faster than Serial.readBytesUntil because it doesn't wait for
		any timeout and does use timedRead();
	*/ 
	
	uint8_t char_count=0;
	
//	buffer[0]=0;	//since we don't know what data is present in that location we clear it. So that we don't have to use
					//any complicated tricks in the while condition.
	
	if (maxlen < 1)
	{
		return 0;
	}
	
	//delay(1000);
	
	while(Serial.available()>0 && char_count < maxlen)
	{
		buffer[char_count]=Serial.read();
		
		if (buffer[char_count]==token)
		{
			buffer[char_count]='\0';		//terminate the string
			return char_count;				//return characters read, doesn't include NULL character.
		}
		
		char_count++;
	}
	
	buffer[char_count]='\0';
	
	return 0;		//returning zero tells that token was not found however we have read whatever characters we could get 
					//from buffer, i.e. emptied the buffer or we just ran out of maxlength specified.
}

/*
uint8_t parse_i2c_cmd( uint8_t * data, uint8_t cmd_type )
{
	int number;
	
	if(cmd_type == I2C_WRITE_CMD)
	{
		data += 8;	//increment data pointer to checking location
		
		if(strncmp(data,"-s",2)==0)
		{
			data += 2;		//increment data pointer to checking location
			
			number = getParam(data,'-');
			if(number == -1)
				return 0;
			else
				i2c_cmd_structure.device_address = number;
		}
		else 
			return 0;
		
		if (number > 0xf)		//it means there where two characters after -s
		{
			data +=2;	//increment data pointer to checking location
		}
		else
		{
			data+=1;	//increment data pointer to checking location
		}
		
		
		
		
		
		if(strncmp(data,"-r",2)==0)
		{
			number = getParam(data,'-');
		
			if(number == -1)
				return 0;
			else
				i2c_cmd_structure.register_address = number;
		}
		else
			return 0;
		
		
		if (number > 0xf)		//it means there where two characters after -s
		{
			data +=2;	//increment data pointer to checking location
		}
		else
		{
			data+=1;	//increment data pointer to checking location
		}
		
		
		
		
		
		if(strncmp(data,"-n",2)==0)
		{
			number = getParam(data,'-');
		
			if(number == -1)
				return 0;
			else
				i2c_cmd_structure.byte_count = number;
		}
		else
			return 0;
			
		if (number > 0xf)		//it means there where two characters after -s
		{
			data +=2;	//increment data pointer to checking location
		}
		else
		{
			data+=1;	//increment data pointer to checking location
		}
		
		
		
	}
	
	else if (cmd_type == I2C_READ_CMD)
	{
		
	}
	
	else if (cmd_type == I2C_PING_CMD)
	{
		
	}
	
}

int getParam(uint8_t * data, uint8_t token)
{
	int number;
	
	if(data[0] >= '0' && data[0] <= '9')
	{
			number = data[0] - 0x30;
	}
	else if (data[0] >= 'a' && data[0] <= 'f')
	{
		number = (data[0] - 0x61) + 0xa;
	}
	else
	{
		return -1;
	}
	
	
	if (data[1] == token)
	{
		return number;
	}
	else if(data[1] >= '0' && data[1] <= '9')
	{
		number = number + ((data[1] - 0x30)<<4);
	}
	else if (data[1] >= 'a' && data[1] <= 'f')
	{
		number = number + (((data[1] - 0x61) + 0xa)<<4);
	}
	else
	{
		return -1;
	}
	
	return number;
	
}*/

uint8_t parse_spi_cmd( char * data, uint8_t cmd_type )
{
	int number;
	char *ptr;
	uint8_t type = HEX;
	
	ptr = strstr(data,"-t");
	if (ptr != NULL)
	{
		if (strncmp(ptr+2,"hex",3) == 0)
		{
			type = HEX;
		}
		else if (strncmp(ptr+2,"dec",3) == 0)
		{
			type = DEC;
		}
		else
		{
			return 0;			//unknown parameter in input type.
		}
		
	}
	
	spi_cmd_structure.type = type;	
	
	
	
	
	if (cmd_type == SPI_WRITE_CMD)
	{
		
		ptr = strstr(data+9,"-n");		//byte count search, here 9 represents length of spi_write
		if (ptr == NULL)		//string not found
		{
			return 0;
		}
		
		number = getParam(ptr+2,'-',type);		//for now token not utilized
		if (number == -1)				//error proper number not found.
		{
			return 0;
		}
		
		spi_cmd_structure.byte_count = (uint8_t)number;
		
		
		ptr = strstr(data+8,"-h");		//data token search
		if (ptr == NULL)		//string not found
		{
			return 0;
		}
		
		number = populateData(ptr+2,8,type, FOR_SPI);			//This will be break down the data and search
		
		spi_cmd_structure.decoded_byte_count = number;
			
	}
	
	else if (cmd_type == SPI_READ_CMD)
	{
		ptr = strstr(data+8,"-n");		//byte count search; here 8 is length of SPI_READ
		if (ptr == NULL)		//string not found
		{
			return 0;
		}
		
		number = getParam(ptr+2,'-',type);		//for now token not utilized
		if (number == -1)				//error proper number not found.
		{
			return 0;
		}
		
		spi_cmd_structure.byte_count = (uint8_t)number;
	}
	
	else if (cmd_type == SPI_CONFIG_CMD)
	{
		//Load default values first
		spi_config.clk_freq_option = 2;
		spi_config.cs_pin = 10;
		spi_config.spiDataOrder = MSBFIRST;
		spi_config.spiMode = SPI_MODE0;
		spi_config.spiClkDiv = SPI_CLOCK_DIV4;
		
		uint8_t found = 0;
		
		ptr = strstr(data+10,"-p");			//ss pin number search, here 10 represents the length of "SPI_config"
		if (ptr != NULL)		//string not found
		{
			found = 1;
			number = getParam(ptr+2,'-',type);		//for now token not utilized
			if (number == -1)				//error proper number not found.
			{
				return 0;
			}
			
			spi_config.cs_pin = (uint8_t)number;
			if ((spi_config.cs_pin >= 11 && spi_config.cs_pin <= 13) || spi_config.cs_pin == A4 || spi_config.cs_pin == A5 || spi_config.cs_pin == 0 || spi_config.cs_pin == 1)
			{
				spi_config.cs_pin = 10;
				Serial.println(F_STR("ERROR! This pin is forbidden for CS"));
				return 0;
			}
			
		}
		
		
		
		
		
		ptr = strstr(data+10,"-x");		//clock divider power search
		if (ptr != NULL)		//string not found
		{
			found = 1;
			number = getParam(ptr+2,'-',type);		//for now token not utilized
			if (number == -1)				//error proper number not found.
			{
				return 0;
			}
			
			spi_config.clk_freq_option = (uint8_t)number;
			
			switch(spi_config.clk_freq_option)
			{
				case 1:spi_config.spiClkDiv = SPI_CLOCK_DIV2;
				break;
				case 2:spi_config.spiClkDiv = SPI_CLOCK_DIV4;
				break;
				case 3:spi_config.spiClkDiv = SPI_CLOCK_DIV8;
				break;
				case 4:spi_config.spiClkDiv = SPI_CLOCK_DIV16;
				break;
				case 5:spi_config.spiClkDiv = SPI_CLOCK_DIV32;
				break;
				case 6:spi_config.spiClkDiv = SPI_CLOCK_DIV64;
				break;
				case 7:spi_config.spiClkDiv = SPI_CLOCK_DIV128;
				break;
				default:
				Serial.println(F_STR("SPI Bad Clock Option"));
				return 0;	//error value out of range
			}
				
		}
		
			
		
		
		ptr = strstr(data+10,"-m");		//SPI mode detect
		if (ptr != NULL)		//string not found
		{
			found = 1;
			number = getParam(ptr+2,'-',type);		//for now token not utilized
			if (number == -1)				//error proper number not found.
			{
				return 0;
			}
			
			spi_config.spiMode = (uint8_t)number;
			
			if (spi_config.spiMode > 3)
			{
				spi_config.spiMode = SPI_MODE0;
				Serial.println(F_STR("SPI Bad mode option"));
				return 0; 
			}
			
			spi_config.spiMode = spi_config.spiMode<<2;
				
		}
		
		
		ptr = strstr(data+10,"-o");		//SPI bit order search, MSB first or LSB first
		if (ptr != NULL)		//string not found
		{
			found = 1;
			if(*(ptr+2) == 'm')
			{
				spi_config.spiDataOrder = MSBFIRST;
			}
			else if (*(ptr+2) == 'l')
			{
				spi_config.spiDataOrder = LSBFIRST;
			}
			else
			{
				Serial.println(F_STR("SPI Bad Bit order option"));
				return 0;		//error, unknown parameter
			}
			
		}
		
		if (found == 0)
		{
			return 0;		//no of the parameters matched.
		}
			
		
		
	}
	
		#ifdef PRINT_DEBUG_MSG
		
		if (cmd_type == SPI_WRITE_CMD || cmd_type == SPI_READ_CMD)
		{
			Serial.print(F_STR("Count:"));
			Serial.println(spi_cmd_structure.byte_count, type);
			Serial.print(F_STR("Decode_Count:"));
			Serial.println(spi_cmd_structure.decoded_byte_count, type);
			Serial.print(F_STR("Decode_data:"));
			for(uint8_t i=0; i <= spi_cmd_structure.decoded_byte_count;i++)
			{
				Serial.print(spi_cmd_structure.data[i],type);
				Serial.print(' ');
			}
		}
		else if(cmd_type == SPI_CONFIG_CMD)
		{
			Serial.print(F_STR("Clk_freq_option:"));
			Serial.println(spi_config.clk_freq_option);
			Serial.print(F_STR("SPI_CLOCK_DIVIDER:"));
			Serial.println(spi_config.spiClkDiv);
			Serial.print(F_STR("SPI_SS_PIN:"));
			Serial.println(spi_config.cs_pin);
			Serial.print(F_STR("SPI_BIT_ORDER:"));
			Serial.println(spi_config.spiDataOrder);
			Serial.print(F_STR("SPI_MODE:"));
			Serial.println(spi_config.spiMode);
		
		}
		
		
		#endif
		
		return 1;
	
}

uint8_t parse_i2c_cmd( char * data, uint8_t cmd_type )
{
	int number;
	char *ptr;
	uint8_t type = HEX;
	
	ptr = strstr(data,"-t");
	if (ptr != NULL)
	{
		if (strncmp(ptr+2,"hex",3) == 0)
		{
			type = HEX;
		}
		else if (strncmp(ptr+2,"dec",3) == 0)
		{
			type = DEC;
		}
		else
		{
			return 0;			//unknown parameter in input type.
		}
		
	}
	
	i2c_cmd_structure.type = type;
	
	
	if(cmd_type == I2C_WRITE_CMD)
	{
		ptr = strstr(data+8,"-s");			//slave address search, here 8 represents the length of "i2cwrite"
		if (ptr == NULL)		//string not found
		{
			return 0;
		}
		
		number = getParam(ptr+2,'-',type);		//for now token not utilized
		if (number == -1)				//error proper number not found.
		{
			return 0;
		}
		
		i2c_cmd_structure.device_address = (uint8_t)number;
		
		
		
		ptr = strstr(data+8,"-r");		//register address search
		if (ptr == NULL)		//string not found
		{
			return 0;
		}
		
		number = getParam(ptr+2,'-',type);		//for now token not utilized
		if (number == -1)				//error proper number not found.
		{
			return 0;
		}
		
		i2c_cmd_structure.register_address = (uint8_t)number;
		
		
		
		ptr = strstr(data+8,"-n");		//byte count search
		if (ptr == NULL)		//string not found
		{
			return 0;
		}
		
		number = getParam(ptr+2,'-',type);		//for now token not utilized
		if (number == -1)				//error proper number not found.
		{
			return 0;
		}
		
		i2c_cmd_structure.byte_count = (uint8_t)number;
		
		
		
		ptr = strstr(data+8,"-h");		//data token search
		if (ptr == NULL)		//string not found
		{
			return 0;
		}
		
		number = populateData(ptr+2,8,type,FOR_I2C);			//This will be break down the data and search
		
		i2c_cmd_structure.decoded_byte_count = number;
		
		
	}
	
	else if (cmd_type == I2C_READ_CMD)
	{
		ptr = strstr(data+7,"-s");			//slave address search, here 7 represents the length of "i2cread"
		if (ptr == NULL)		//string not found
		{
			return 0;
		}
		
		number = getParam(ptr+2,'-',type);		//for now token not utilized
		if (number == -1)				//error proper number not found.
		{
			return 0;
		}
		
		i2c_cmd_structure.device_address = (uint8_t)number;
		
		
		
		ptr = strstr(data+7,"-r");		//register address search
		if (ptr == NULL)		//string not found
		{
			return 0;
		}
		
		number = getParam(ptr+2,'-',type);		//for now token not utilized
		if (number == -1)				//error proper number not found.
		{
			return 0;
		}
		
		i2c_cmd_structure.register_address = (uint8_t)number;
		
		
		
		ptr = strstr(data+7,"-n");		//byte count search
		if (ptr == NULL)		//string not found
		{
			return 0;
		}
		
		number = getParam(ptr+2,'-',type);		//for now token not utilized
		if (number == -1)				//error proper number not found.
		{
			return 0;
		}
		
		i2c_cmd_structure.byte_count = (uint8_t)number;
	}
	
	else if (cmd_type == I2C_PING_CMD)
	{
		ptr = strstr(data+7,"-s");			//slave address search, here 7 represents the length of "i2cping"
		if (ptr == NULL)		//string not found
		{
			return 0;
		}
		
		number = getParam(ptr+2,'-',type);		//for now token not utilized
		if (number == -1)				//error proper number not found.
		{
			return 0;
		}
		
		i2c_cmd_structure.device_address = (uint8_t)number;
	}
	
	else								//Un-known command type.
	{ 
		return 0;
	}
	
	#ifdef PRINT_DEBUG_MSG 
	
	
	Serial.print(F_STR("Device:"));
	Serial.println(i2c_cmd_structure.device_address, type);
	Serial.print(F_STR("Register:"));
	Serial.println(i2c_cmd_structure.register_address, type);
	Serial.print(F_STR("Count:"));
	Serial.println(i2c_cmd_structure.byte_count, type);
	Serial.print(F_STR("Decode_Count:"));
	Serial.println(i2c_cmd_structure.decoded_byte_count, type);
	Serial.print(F_STR("Decode_data:"));
	for(uint8_t i=0; i <= i2c_cmd_structure.decoded_byte_count;i++)
	{	
		Serial.print(i2c_cmd_structure.data[i],type);
		Serial.print(' ');
	}
	
	#endif
	
	return 1;
}

int getParam(char * data, uint8_t token, const uint8_t type)
{
	int number;
	
	if (type == HEX)
	{
		if(data[0] >= '0' && data[0] <= '9')		//check if first character is a hex number
		{
			number = data[0] - 0x30;
		}
		else if (data[0] >= 'a' && data[0] <= 'f')
		{
			number = (data[0] - 0x61) + 0xa;
		}
		else
		{
			return -1;
		}
		
		
		if (data[1] == '-' || data[1] == '\0' || data[1] == ' ' || data[1] == token)		//check if second character is one of the terminators or a number
		{
			return number;
		}
		else if(data[1] >= '0' && data[1] <= '9')
		{
			number = (number<<4) + (data[1] - 0x30);
		}
		else if (data[1] >= 'a' && data[1] <= 'f')
		{
			number = (number<<4) + ((data[1] - 0x61) + 0xa);
		}
		else
		{
			return -1;
		}
		
		
		if (data[2] == '-' || data[2] == '\0' || data[2] == ' ' || data[2] == token)	//check after second number if there is any terminator if not say error.
		{															//prevents spurious characters.
			return number;
		}
		else
		{
			return -1;
		}
	}
	
	else if(type == DEC)
	{
		if(data[0] >= '0' && data[0] <= '9')		//check if first character is a hex number
		{
			number = data[0] - 0x30;
		}
		else
		{
			return -1;
		}
		
		
		if (data[1] == '-' || data[1] == '\0' || data[1] == ' ' || data[1] == token)		//check if second character is one of the terminators or a number
		{
			return number;
		}
		else if(data[1] >= '0' && data[1] <= '9')
		{
			number = (number*10) + (data[1] - 0x30);
		}
		else
		{
			return -1;
		}
		
		if (data[2] == '-' || data[2] == '\0' || data[2] == ' ' || data[2] == token)		//check if third character is one of the terminators or a number
		{
			return number;
		}
		else if(data[2] >= '0' && data[2] <= '9')
		{
			number = (number*10) + (data[2] - 0x30);
		}
		else
		{
			return -1;
		}
		
		if (data[3] == '-' || data[3] == '\0' || data[3] == ' ' || data[3] == token)	//check after third number if there is any terminator if not say error.
		{															//prevents spurious characters.
			if (number <= 255)
			{
				return number;
			}
			else return -1;					//number specified bigger than A byte.
			
		}
		else
		{
			return -1;
		}
		
		
	}
	
	else
	{
		return -1;		//unknown type specified.
	}
	
	
}

uint8_t populateData(char *data, uint8_t max_bytes, uint8_t type, uint8_t i2c_spi)
{
	int number;
	uint8_t count=0;
	
		number = getParam(data,',',type);
		if (i2c_spi == FOR_I2C)
		{
			i2c_cmd_structure.data[count] = (uint8_t)number;
		}
		else if (i2c_spi == FOR_SPI)
		{
			spi_cmd_structure.data[count] = (uint8_t)number;
		}
		else
		{
			return 0; //error, unknown parameter
		}
		
		
		
		while (number != -1 && count < max_bytes)
		{
			if (type == HEX)
			{
				/*
				if (number > 0xf)
				{
					data+=3;
				}
				else
				{
					data+=2;
				}
				*/
			//the problem that I was facing with the above code is that when i wrote i2cwrite -s10 -r0 -n2 -h01,a0
			//it was not able to anticipate that i wrote 01 instead of 1 becuase of which it executed the above else
			//block but it was not correct.
				while(*data!=',' && *data != '\0')
				{
					data++;
				}
				data++;
				
				
				
			}
			
			
			else if (type == DEC)
			{
				/*
				if (number > 99)
				{
					data+=4;
				}
				else if (number > 9)
				{
					data+=3;
				}
				
				else
				{
					data+=2;
				}
				*/
				while(*data!=',' && *data != '\0')		//break if detected a comma or a null pointer (end of input string)
				{
					data++;
				}
				data++;		//found address is for comma, but we need to give the next base address.
			}
			
			count++;
			number = getParam(data,',',type);
			if (i2c_spi == FOR_I2C)
			{
				i2c_cmd_structure.data[count] = (uint8_t)number;
			}
			else if (i2c_spi == FOR_SPI)
			{
				spi_cmd_structure.data[count] = (uint8_t)number;
			}
			else
			{
				return 0; //error, unknown parameter
			}
			
			
			
		}
	
	
	
	#ifdef PRINT_DEBUG_MSG 
	
	Serial.print(F_STR("Decoded byte count:"));
	Serial.print(count);
	
	#endif
	
	return count;
	
}


/**
 * Divides a given PWM pin frequency by a divisor.
 * 
 * The resulting frequency is equal to the base frequency divided by
 * the given divisor:
 *   - Base frequencies:
 *      o The base frequency for pins 3, 9, 10, and 11 is 31250 Hz.
 *      o The base frequency for pins 5 and 6 is 62500 Hz.
 *   - Divisors:
 *      o The divisors available on pins 5, 6, 9 and 10 are: 1, 8, 64,
 *        256, and 1024.
 *      o The divisors available on pins 3 and 11 are: 1, 8, 32, 64,
 *        128, 256, and 1024.
 * 
 * PWM frequencies are tied together in pairs of pins. If one in a
 * pair is changed, the other is also changed to match:
 *   - Pins 5 and 6 are paired on timer0
 *   - Pins 9 and 10 are paired on timer1
 *   - Pins 3 and 11 are paired on timer2
 * 
 * Note that this function will have side effects on anything else
 * that uses timers:
 *   - Changes on pins 3, 5, 6, or 11 may cause the delay() and
 *     millis() functions to stop working. Other timing-related
 *     functions may also be affected.
 *   - Changes on pins 9 or 10 will cause the Servo library to function
 *     incorrectly.
 * 
 * Thanks to macegr of the Arduino forums for his documentation of the
 * PWM frequency divisors. His post can be viewed at:
 *   http://forum.arduino.cc/index.php?topic=16612#msg121031
 */
uint8_t setPwmFrequency(int pin, int divisor)
{
	byte mode;
	if(pin == 5 || pin == 6 || pin == 9 || pin == 10)
	{
		switch(divisor)
		{
			case 1: mode = 0x01; break;
			case 8: mode = 0x02; break;
			case 64: mode = 0x03; break;
			case 256: mode = 0x04; break;
			case 1024: mode = 0x05; break;
			default: return 0;
		}
		if(pin == 5 || pin == 6)
		{
			TCCR0B = TCCR0B & 0b11111000 | mode;
		} else
		{
			TCCR1B = TCCR1B & 0b11111000 | mode;
		}
	}
	else if(pin == 3 || pin == 11)
	{
		switch(divisor)
		{
			case 1: mode = 0x01; break;
			case 8: mode = 0x02; break;
			case 32: mode = 0x03; break;
			case 64: mode = 0x04; break;
			case 128: mode = 0x05; break;
			case 256: mode = 0x06; break;
			case 1024: mode = 0x07; break;
			default: return 0;
		}
		TCCR2B = TCCR2B & 0b11111000 | mode;
	}
	else
	{
		//not necessary but still implementing pin check
		return 0;
	}
	
	
	return 1;
}