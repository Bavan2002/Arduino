/*
* Code.cpp
*
* Created: 6/30/2024 4:51:37 PM
* Author : Yaseema Rusiru
*/

#define F_CPU 8000000UL

#include <avr/io.h>   //Include AVR io
#include <avr/interrupt.h> //Include std interrupt
#include <util/delay.h>  //Include delay functions
#include <stdio.h>  //Include standard I/O
#include <avr/io.h> 
#include <avr/pgmspace.h> 
#include <avr/wdt.h> 
#include <avr/eeprom.h> 
#include <util/twi.h> 
#include <stdlib.h> 
#include <avr/interrupt.h> 
#include <avr/sleep.h> 
#include <avr/power.h> 
#include <avr/boot.h>

#define I2C_FREQ 100000L
#define AS5600_ADDRESS 0x36 // Default device I2C address
#define SCL  PINC5  //Define I2C pins
#define SDA  PINC4

/* byte 1 : angle[15:8]
byte 2 : angle[7:0]
byte 3: turns[7:0] */

uint8_t byte1 = 0;
uint8_t byte2 = 0;
uint32_t byte3 = 0;

uint8_t q=0;
uint8_t preQ = 0;

uint16_t int1;
uint16_t int2;
float angle;

void delay_func(uint8_t count) {   //delay function
 volatile uint8_t delay = 0;
 while (delay < count) {
  delay++;
 }
}

void startI2C(uint32_t address) {    //start I2C
 TWCR = (1<<TWSTA)|(1<<TWEN)|(1<<TWINT); //Enable I2C and generate START condition
 while(!(TWCR&(1<<TWINT))); //Wait until I2C finish its current work
 TWDR = address;  //Write SLA+W in I2C data register
 TWCR = (1<<TWEN)|(1<<TWINT); // Enable I2C and clear interrupt flag
 while (!(TWCR&(1<<TWINT))); // Wait until I2C finish its current work
}

uint8_t getAngle(uint8_t reg){
 uint8_t byte;
 
 // Start I2C transmission to write the register address
 startI2C((AS5600_ADDRESS << 1) | 0);     // AS5600 address with write bit

 TWDR = reg;   //write register address to TWDR register
 TWCR = (1<<TWEN)|(1<<TWINT);  //clear interrupt flag
 while (!(TWCR&(1<<TWINT)));   //wait until interrupt flag is cleared

 // Start I2C transmission to read the data
 startI2C((AS5600_ADDRESS << 1) | 1);   // Address with read bit
 
 // Read byte 
 TWCR = (1<<TWEN)|(1<<TWINT);  //clear interrupt flag again to start reading
 while (!(TWCR&(1<<TWINT)));  //wait until read operation is complete
 byte = TWDR;    //read the received byte in TWDR register

 // Stop I2C transmission 
 TWCR = (1<<TWSTO)|(1<<TWINT)|(1<<TWEN); //generate stop condition
 while (TWCR&(1<<TWSTO));   //wait until stop condition is executed
 
 return byte;
}

void USART_init(uint16_t ubrr) {
    UBRR0H = (ubrr>>8);
    UBRR0L = ubrr;
    UCSR0B = (1<<TXEN0);  // Enable transmitter
    UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);  // Set frame format: 8 data bits, 1 stop bit
}

void USART_send(char data) {
    while (!(UCSR0A & (1<<UDRE0)));  // Wait until buffer is empty
    UDR0 = data;  // Put data into buffer and send
}

void USART_sendString(const char* str) {
    while (*str) {
        USART_send(*str++);
    }
}

int main(void){
 DDRC |= (1<<SDA)|(1<<SCL);   //set SCL and SDA as output
 PORTC |= (1<<SDA)|(1<<SCL);  //set SCL and SDA high

 TWSR = 0x00; // Set pre-scaler to 1
 TWBR = ((F_CPU / I2C_FREQ) - 16) / 2;   //initialize i2c frequency

 ADCSRA = 0;  //disable ADC
 PRR = (1<<PRTIM1)|(1<<PRTIM0)|(1<<PRTIM2);  //disable timers
 
 USART_init(51);  // Initialize USART with baud rate 9600 (assuming 8 MHz clock)

 sei();  //global interrupt enable
 
 while (1) {
  byte1 = getAngle(0x0D); //get high byte
  byte2 = getAngle(0x0C); //get low byte

  int1 = (int) byte1;
  int2 = (int) byte2;

  float raw_angle = (int2<<8)|int1; //raw angle calculation - between 0 - 4096 (12 bit value)
  angle = raw_angle*360/4096; //to get angle in 0-360 range
  
  //update byte3
  if (0<=angle && angle<=90) q = 1;
  else if (90<angle && angle<=180) q = 2;
  else if (180<angle && angle<=270) q = 3;
  else if (270<angle && angle<360) q = 4;
  if (q!=preQ) {
   if (q==1 && preQ==4) byte3++; //increase turns count
   if (q==4 && preQ==1) byte3--; //decrease turns count
   preQ = q;

   // Send the number of turns to serial output
   char buffer[10];
   sprintf(buffer, "Turns: %d\n", byte3);
   USART_sendString(buffer);
  }
   
  delay_func(100);
 }
 return 0;
}
