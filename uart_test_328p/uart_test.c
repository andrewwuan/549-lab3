/* some includes */
#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <stdio.h>
#include <util/twi.h>
#include <stdlib.h>

#define BAUD 115200
#include <util/setbaud.h>

#define ERROR -1
#define SUCCESS 0

void uart_init(void) {
   UBRR0H = UBRRH_VALUE;
   UBRR0L = UBRRL_VALUE;

#if USE_2X
   UCSR0A |= _BV(U2X0);
#else
   UCSR0A &= ~(_BV(U2X0));
#endif

   UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); /* 8-bit data */
   UCSR0B = _BV(RXEN0) | _BV(TXEN0);   /* Enable RX and TX */
}

void uart_putchar(char c) {
   loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
   UDR0 = c;
}

char uart_getchar(void) {
   loop_until_bit_is_set(UCSR0A, RXC0); /* Wait until data exists. */
   return UDR0;
}

FILE uart_output = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
FILE uart_input = FDEV_SETUP_STREAM(NULL, uart_getchar, _FDEV_SETUP_READ);
FILE uart_io = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);


// I2C code
void TWIInit(void)
{   
   printf("TWI init\r\n");

   //set SCL to 400kHz
   TWSR = 0x00;
   TWBR = 0x0C;
   //enable TWI
   TWCR = (1<<TWEN);

}

// Start signal
void TWIStart(void)
{
   printf("In TWIStart\r\n");
   TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
   printf("TWCR=%b\n", TWCR);
   while ((TWCR & (1<<TWINT)) == 0);
   printf("Leaving TWIStart\r\n");
}

// Stop signal
void TWIStop(void)
{
   TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
}

// I2C write
void TWIWrite(uint8_t u8data)
{
   TWDR = u8data;
   TWCR = (1<<TWINT)|(1<<TWEN);
   while ((TWCR & (1<<TWINT)) == 0);
}

// I2C read
uint8_t TWIReadACK(void)
{
   TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
   while ((TWCR & (1<<TWINT)) == 0);
   return TWDR;
}

// I2C read
// (read byte with NACK)
uint8_t TWIReadNACK(void)
{
   TWCR = (1<<TWINT)|(1<<TWEN);
   while ((TWCR & (1<<TWINT)) == 0);
   return TWDR;
}

// I2C get status
uint8_t TWIGetStatus(void)
{
   uint8_t status;
   //mask status
   status = TWSR & 0xF8;
   return status;
}

// Write page
uint8_t EEWriteData(uint8_t reg_addr)
{
   printf("in EEWriteData\r\n");
   TWIStart();
   if (TWIGetStatus() != 0x08)
      return ERROR;
   printf("after start\r\n");

   // write slave addr
   TWIWrite((uint8_t)26);
   if (TWIGetStatus() != 0x18)
      return ERROR;
   printf("after write\r\n");

   // Write register addr
   TWIWrite(reg_addr);
   if (TWIGetStatus() != 0x18)
      return ERROR;
   printf("after write\r\n");

   // Send stop code
   TWIStop();
   return SUCCESS;
}

uint8_t EEReadData(uint8_t *u8data)
{
   printf("in EEReadData\r\n");
   TWIStart();
   if (TWIGetStatus() != 0x08)
      return ERROR;
   printf("after start\r\n");

   // write slave addr
   TWIWrite((uint8_t)27);
   if (TWIGetStatus() != 0x18)
      return ERROR;
   printf("after write\r\n");

   *u8data = TWIReadNACK();
   if (TWIGetStatus() != 0x18)
      return ERROR;
   printf("after read\r\n");

   // Send stop code
   TWIStop();
   return SUCCESS;
}

//Simple Wait Function
void Wait()
{
   _delay_ms(1000);

}

void degree_0 () {
  OCR1B=1800;   //0 degree
  Wait();
}


void degree_90 () {
  OCR1B=3000;  //90 degree
  Wait();
}

void degree_135 () {
  OCR1B=3800;  //135 degree
  Wait();
}

void degree_180 () {
  OCR1B=4500;  //180 degree
  Wait();
}


int main(void)
{
   cli();

   /* Setup uart port */
   uart_init();
   stdout = &uart_output;
   stdin  = &uart_input;

   // Setup ports
   DDRB |= (1<<1) | (1<<0);
   PORTB |= (1<<0);
   PORTB &= ~(1<<1);

   /* Setup TWI for I2C */
   TWIInit();

   // Configure Servo
   TCCR1A=(1<<COM1B1)|(1<<COM1A1)|(1<<WGM11);        // NON Inverted PWM
   TCCR1B=(1<<WGM13)|(1<<WGM12)|(1<<CS11);   //PRESCALER=64 MODE 14(FAST PWM)
   ICR1=39999;  // fPWM=50Hz (Period = 20ms Standard).
   DDRB|=(1<<PB2);   //PWM Pins as Out

   sei();

   /* Print hello  */
   printf("Hello world!\r\n");

   /* Pointer for EEReadData */
   uint8_t* idp = (uint8_t *)malloc(8);

   while(1) {
      
      // EEWriteData((uint8_t)81);
      // EEReadData(idp);
      // printf("Product ID is %c\r\n", *idp);

      degree_0();
      degree_180();

      // PORTB ^= 0x01;
   }
}
