/*
 * core.cpp
 *
 *  Created on: 19 de Jan de 2019
 *      Author: Eduardo Lacerda Campos
 */


#include "stdlib.h"
#include "stdint.h"
#include <stdio.h>
#include <inttypes.h>
//#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <string.h>

#include <avr/io.h>
#include <util/delay.h>
#include "OV7670.h"
#include "MCP23017.h"

#include <Wire.h>
#include "Usart.h"
#include "DS3231.h"
#include "TimerOne.h"

#include "FAT/SDCard.h"
#include "FAT/FAT.h"
#include "FAT/File.h"
#include "FAT/SPISD.h"
#include "Battery.h"
#include "NEO6M.h"

#include "Error.h"

NEO6M GPS;

SDCard disk(&PORTB, &DDRB, PB2);

FAT fs(&disk);
File root(&fs);
File file_log(&fs);
File Sensor_log(&fs);
File Image(&fs);

DS3231 rtc;
OV7670 Can;
MCP23017 GPIO(0);
uint8_t Fig_Count=0;

BATTERY Battery;

uint64_t time=0;
uint16_t count_photo=0;

#include <avr/wdt.h>

#define CAN_POWER_UP() GPIO.Clear_bit(2,GPIOA)
#define CAN_POWER_DOWN() GPIO.Set_bit(2,GPIOA)
#define SD_POWER_UP() PORTB &= ~(1<<PB0)//enable SD_card
#define SD_POWER_DOWN() PORTB |= (1<<PB0) //disable SD_Card
#define PARACHUTE_DEPLOY() PORTB |= (1<<PB1)
#define PARACHUTE_SETUP() DDRB |= (1<<PB0); PORTB &= ~(1<<PB1)


/******************************************************************
 * Soft reset
 ******************************************************************/

#define soft_reset()        \
do                          \
{                           \
    wdt_enable(WDTO_4S);  \
    for(;;)                 \
    {                       \
    }                       \
} while(0)

void WDT_off(void)
{
	cli();
	wdt_reset();
	/* Clear WDRF in MCUSR */
	MCUSR &= ~(1<<WDRF);
	/* Write logical one to WDCE and WDE */
	/* Keep old prescaler setting to prevent unintentional
	time-out */
	WDTCSR |= (1<<WDCE) | (1<<WDE);
	/* Turn off WDT */
	WDTCSR = 0x00;

}


/******************************************************************
 *
 ******************************************************************/

void Erro_Action(void){

	file_log.close();
	Sensor_log.close();
}

/******************************************************************
 * System Timer
 ******************************************************************/
void timerIsr()
{

	time++;

}

/******************************************************************
 * SD Card routine to inform the error
 ******************************************************************/

void handle_error()
{
    switch(disk.get_error()){
    case SDCard::Error::CMD0:
    	printf_P(PSTR("timeout error for command CMD0\r\n"));
        break;
    case SDCard::Error::CMD8:
    	printf_P(PSTR("CMD8 was not accepted - not a valid SD card\r\n"));
        break;
    case SDCard::Error::ACMD41:
    	printf_P(PSTR("card's ACMD41 initialization process timeout\r\n"));
        break;
    case SDCard::Error::CMD58:
    	printf_P(PSTR("card returned an error response for CMD58 (read OCR)\r\n"));
        break;
    case SDCard::Error::CMD24:
    	printf_P(PSTR("card returned an error response for CMD24 (write block)\r\n"));
        break;
    case SDCard::Error::READ_TIMEOUT:
    	printf_P(PSTR("timeout while waiting for start of read data\r\n"));
        break;
    default:
        printf_P(PSTR("Unknown error. Code %x\r\n"), (uint8_t)disk.get_error());
        break;
    }
}

/******************************************************************
 * Routine to save the image read from the camera
 ******************************************************************/
void Save(uint8_t b){
//	USART.writeByte(b);
	Image.write(&b, 1);
}

/******************************************************************
 * Read the port B from the MCP23017
 ******************************************************************/
uint8_t Read_GPIO(){

	uint8_t val;

	if (!GPIO.Read_Byte(&val,GPIOB))
	{
		Failure(5);
	}

	return val;
}

/******************************************************************
 * Define the instruction to the printf or printfP routines
 ******************************************************************/
int Std_putchar(char c, FILE *stream) {
	USART.writeByte(c);
    file_log.write((uint8_t*)&c, 1);
    return 0;
}

/******************************************************************
 * Initialize SD Card
 ******************************************************************/


void SD_Init(void){

	//Operating Voltage: 2.7V to 3.6V

	DDRB |= (1<<PB0);
	SD_POWER_DOWN();
	disk.select();
	_delay_ms(2000);
	disk.deselect();
	SD_POWER_UP();
	_delay_ms(2000);

    printf_P(PSTR("Initializing SD card...\r\n"));
    if(disk.init()){
    	printf_P(PSTR("Card connected!\r\n"));
    } else {
    	printf_P(PSTR("Card initialization failed.\r\n"));
        handle_error();
        Failure(SDCARD_INIT);
    }

    SPI_SD::set_speed();

    printf_P(PSTR("\nMounting FAT Filesystem...\r\n"));
    if(fs.mount()){
    	printf_P(PSTR("Filesystem mounted!\r\n"));
    	printf_P(PSTR("Filesystem Type FAT%i\r\n"), fs.get_type());
//    	printf_P(PSTR("Free Memory %i\n"), freeRam());
    } else {
    	printf_P(PSTR("Mount error.\r\n"));
        handle_error();
        Failure(SDCARD_INIT);
    }
    printf_P(PSTR("\nOpening filesystem root...\r\n"));
    if(root.open_root()){
    	printf_P(PSTR("Root is open\r\n"));
//    	printf_P(PSTR("Free Memory %i\n"), getFreeMCUMemory());
    } else {
    	printf_P(PSTR("Unable to open root\r\n"));
        handle_error();
        Failure(SDCARD_INIT);
    }

    if(!file_log.open(root, "LOG.TXT", File::O_CREAT | File::O_RDWR | File::O_APPEND )){

    	printf_P(PSTR("Unable to open LOG.TXT\r\n"));
    	Failure(FILE_OPEN);
    }
}

/******************************************************************
 * Restart SD Card
 ******************************************************************/
void SD_Restart() {

	printf_P(PSTR("Reseting\r\n"));
	asm volatile ("  jmp 0");

}



/******************************************************************
 * Main setup
 ******************************************************************/
inline void setup() {

	WDT_off();

	_delay_ms(1000);

	USART.begin(9600,70);

	//BUG - On the AVR, using the serial port direct can cause a BUG in the stdout
	//To avoid this BUG, first use the stdout
	stdout = fdevopen(Std_putchar, NULL);
	//    stdin  = fdevopen(NULL, uart_getchar);

	printf_P(PSTR("Serial OK\r\n"));

	printf_P(PSTR("Free Memory %i\r\n"), getFreeMCUMemory());

	SD_Init();

    printf_P(PSTR("Log Init-------------------------------------------------\r\n"));

    printf_P(PSTR("Free Memory %i\r\n"), getFreeMCUMemory());

	Wire.begin();

	printf_P(PSTR("I2C Init OK\r\n"));

	if(rtc.Initialize(&Wire)==false)
	{
		Failure(RCT_INIT);
	}

	if(rtc.Read()==false)
	{
		printf_P(PSTR("Real Time Clock read fail\r\n"));
		Failure(RCT_READ);
	}

	printf_P(PSTR("Real Time Clock init OK\r\n"));

	printf_P(PSTR("Time %d/%d/%d %d:%d:%d\r\n"),rtc.y+2000,rtc.m,rtc.d,rtc.hh,rtc.mm,rtc.ss);

	//----------------------------------------------------------
	if (!GPIO.Initialize(&Wire))
	{
		printf_P(PSTR("GPIO init FAIL\r\n"));
		Failure(GPIO_INIT);
	}

	//Set GPIO Direction
	if (!GPIO.Set_Port_Direction(0b11111100,GPIOA))
	{
		printf_P(PSTR("GPIO init FAIL\r\n"));
		Failure(GPIO_INIT);
	}
	if (!GPIO.Write_Byte(0b00000001,GPIOA))
	{
		printf_P(PSTR("GPIO init FAIL\r\n"));
		Failure(GPIO_INIT);
	}

	printf_P(PSTR("GPIO init OK\r\n"));
	//----------------------------------------------------------
	CAN_POWER_UP();
	if (!Can.Initialize(&Wire))
	{
		printf_P(PSTR("Camera init FAIL\r\n"));
		Failure(CAN_INIT);

	}

	Can.old=false;
	Can.Save_Image = &Save;
	Can.Read_GPIO = &Read_GPIO;

	//this value gives a nice result
	Can.Contrast(0x50);

	printf_P(PSTR("Camera init OK\r\n"));



	//----------------------------------------------------------
	//TODO - Colocar na memoria flash
    if(!Sensor_log.open(root, "SENSOR.TXT", File::O_CREAT | File::O_RDWR | File::O_APPEND)){

    	printf_P(PSTR("Unable to open SENSOR.TXT\r\n"));
    	handle_error();
    	Failure(FILE_OPEN);
    }
	Sensor_log.close();

    printf_P(PSTR("File opened SENSOR.TXT\r\n"));

    printf_P(PSTR("Free Memory %i\r\n"), getFreeMCUMemory());

    //----------------------------------------------------------
    Battery.initialize();

    printf_P(PSTR("Battery Voltage Sensor OK\r\n"));

    //----------------------------------------------------------
    file_log.sync();

    count_photo = eeprom_read_word(0);

    //----------------------------------------------------------
	while(!GPS.Initialize(&USART)){
		printf_P(PSTR("GPS Fail\r\n"));
	}

	while(!GPS.is_position_valid()){
		GPS.Status_Read();
		printf_P(PSTR("Waiting for the GPS position\r\n"));
	}

	printf_P(PSTR("GPS OK\r\n"));


	//Parashute Setup------------------------------------------------

	Timer1.initialize();
	Timer1.attachInterrupt(timerIsr);

	uint64_t counter=0;

	while(counter<2)
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
			counter=time;
			_delay_ms(100);
		}

	printf_P(PSTR("Timer OK\r\n"));

	PARACHUTE_SETUP();

	printf_P(PSTR("Setup Finished \r\n"));
	//send to the communication module that the CubeSat is ready to fly
	while(!GPIO.Set_bit(1,GPIOA));

	printf_P(PSTR("Free Memory %i\r\n"), getFreeMCUMemory());
	printf_P(PSTR("Ready to fly\r\n"));

//	DateTime rtc_date(2019,1,19,12,03,00);
//	rtc.Adjust_Time(rtc_date);


}

void Sensor_read()
{

    double Bat = Battery.Read_Voltage();
    rtc.Read();
    rtc.Read_Temperature();
    double rtc_temp = rtc.Get_Temperature();

//    printf_P(PSTR("Reading GPS Status \r\n"));
    GPS.Status_Read();
    char status[15];
    if(GPS.is_position_valid())
    {
    	strcpy_P(status, PSTR("GPS OK"));
    	printf_P(PSTR("GPS position is valid\r\n"));
    }
    else
    {
    	strcpy_P(status,PSTR("GPS Fail"));
    	printf_P(PSTR("GPS position is invalid\r\n"));
    }

    GPS.ECEF_Read();
    GPS.NED_VEL_Read();

    char buffer[200];
    sprintf_P(buffer, PSTR("%d/%d/%d %d:%d:%d ; %.4f ; %.4f ; %d ; %s ;  %li ; %li ; %li ; %li ; %li ; %li \r\n"),rtc.y+2000,rtc.m,rtc.d,rtc.hh,rtc.mm,rtc.ss,rtc_temp,Bat,getFreeMCUMemory(),status,GPS.ECEF_X,GPS.ECEF_Y,GPS.ECEF_Z,GPS.NED_Vel_N,GPS.NED_Vel_E,GPS.NED_Vel_D);
	Sensor_log.write((const uint8_t*)buffer, strlen(buffer));
	//printf_P(PSTR("%d/%d/%d %d:%d:%d \r\n"),rtc.y+2000,rtc.m,rtc.d,rtc.hh,rtc.mm,rtc.ss);
	printf(buffer);
}

void Save_sensores(){
	char buffer[12];
	sprintf_P(buffer, PSTR("SENSOR.TXT"));
    if(!Sensor_log.open(root, buffer, File::O_CREAT | File::O_RDWR | File::O_APPEND)){

    	printf_P(PSTR("Unable to open SENSOR.TXT\r\n"));
    	handle_error();
    	Failure(FILE_OPEN);
    }
    else{
    	printf_P(PSTR("Save sensor data\r\n"));
		Sensor_read();
		Sensor_log.close();
    }
}

void Save_image(){

	char buffer[12];
	sprintf_P(buffer, PSTR("i%d.bmp"),count_photo);
    if(!Image.open(root, buffer, File::O_CREAT | File::O_RDWR )){

    	printf_P(PSTR("Unable to create the image\r\n"));
    	handle_error();
    	Failure(FILE_OPEN);
    }
    else{
    	if(!Can.Capture()){
    		printf_P(PSTR("Camera failed to capture\r\n"));
    		Failure(CAN_FAIL);
    	}
    	else
    	{
    		printf_P(PSTR("Saving image\r\n"));
    		Can.Read_and_Save_Image();
    	}

    	printf_P(PSTR("Closing the image %d\r\n"), count_photo);

    	if(!Image.close())
    	{
    		printf_P(PSTR("Failed to close the image %d\r\n"), count_photo);
    		handle_error();
    		Failure(FILE_CLOSE);
    	}
    	else{
    		printf_P(PSTR("Image saved OK\r\n"));
    		count_photo++;
    		eeprom_write_word(0,count_photo);
    	}

    }

}

void Sync_Log(){

    if(!file_log.sync()){
    	printf_P(PSTR("Fail to Sync\r\n"));
		handle_error();
		Failure(FILE_SYNC);
    }

}


void Free_fall(){

	GPS.NED_VEL_Read();
	static uint64_t counter=0;
	uint64_t delta_time;

	if((GPS.NED_Vel_D/100.0) > 10.0){
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
			delta_time = time - counter;
		}

		if(delta_time>120)
			PARACHUTE_DEPLOY();
	}
	else
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
			counter = time;
		}

}

void loop() {


	Save_sensores();

	Free_fall();

	Save_image();

	Sync_Log();


	//One minute of delay
	_delay_ms(5000);

}


int main(void)
{
	setup();

	for (;;)
		loop();

	return 0;
}





