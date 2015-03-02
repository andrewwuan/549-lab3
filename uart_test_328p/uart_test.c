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


#define ERROR_STR(str) printf(str);

// I2C code
void TWIInit(void)
{   
   printf("Enter TWI init\r\n");

   //set SCL to 400kHz
   TWSR = 0x00;
   TWBR = 0x0C;

   //enable TWI
   TWCR = (1<<TWEN);

   printf("Leaving TWI init, TWCR=%d\r\n", TWCR);

}

#define START 0x08
#define MT_SLA_ACK 0x18
#define MT_DATA_ACK 0x28
#define MR_SLA_ACK 0x40


// Start signal
void TWIStart(void)
{
   printf("Enter TWIStart\r\n");
   TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);

   // printf("TWCR=%d (should be %d)\r\n", TWCR, (1<<TWINT)|(1<<TWSTA)|(1<<TWEN));

   while (!(TWCR & (1<<TWINT)));

   if ((TWSR & 0xF8) != START)
      ERROR_STR("TWI Start failed\r\n");
   printf("Leaving TWIStart\r\n");
}

// Stop signal
void TWIStop(void)
{
   printf("Enter TWIStop\r\n");
   TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
   printf("Leaving TWIStop\r\n");
}

// I2C write
void TWISLAWWrite(uint8_t u8data)
{
   printf("Enter TWISLAWWrite\r\n");
   TWDR = u8data;
   TWCR = (1<<TWINT)|(1<<TWEN);
   while (!(TWCR & (1<<TWINT)));

   if ((TWSR & 0xF8) != MT_SLA_ACK)
      ERROR_STR("TWISLAWWrite failed\r\n");
   printf("Leaving TWISLAWWrite\r\n");
}

// I2C write
void TWISLARWrite(uint8_t u8data)
{
   printf("Enter TWISLARWrite\r\n");
   TWDR = u8data;
   TWCR = (1<<TWINT)|(1<<TWEN);
   while (!(TWCR & (1<<TWINT)));

   if ((TWSR & 0xF8) != MR_SLA_ACK)
      ERROR_STR("TWISLARWrite failed\r\n");
   printf("Leaving TWISLARWrite\r\n");
}

// I2C write
void TWIDataWrite(uint8_t u8data)
{
   printf("Enter TWIDataWrite\r\n");
   TWDR = u8data;
   TWCR = (1<<TWINT)|(1<<TWEN);
   while (!(TWCR & (1<<TWINT)));

   if ((TWSR & 0xF8) != MT_DATA_ACK)
      ERROR_STR("TWIDataWrite failed\r\n");
   printf("Leaving TWIDataWrite\r\n");
}

// I2C read
uint8_t TWIReadACK(void)
{
   printf("Enter TWIReadACK\r\n");
   TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
   while ((TWCR & (1<<TWINT)) == 0);

   printf("Leaving TWIReadACK\r\n");
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
   printf("Enter EEWriteData\r\n");
   TWIStart();

   // write slave addr
   TWISLAWWrite((uint8_t)0x26);

   // Write register addr
   TWIDataWrite(reg_addr);

   // Send stop code
   TWIStop();

   printf("Leaving EEWriteData\r\n");
   return SUCCESS;
}

uint8_t EEReadData(uint8_t *u8data)
{
   printf("Enter EEReadData\r\n");

   TWIStart();

   // write slave addr
   TWISLARWrite((uint8_t)0x27);

   *u8data = TWIReadACK();
   if (TWIGetStatus() != 0x50)
      ERROR_STR("TWIReadACK failed\r\n");

   // Send stop code
   TWIStop();

   printf("Leaving EEReadData");
   return SUCCESS;
}

//Simple Wait Function
void Wait(int interval)
{
   int i;
   for ( i = 0 ; i < interval; i++) {
      _delay_ms(20);
   }

}

void rotate(int degree, int interval) {
   OCR1B = 1800 + degree * 15;
   Wait(interval);
}


// Switch logic
#define SWITCH_SPEED 20

int mode = 0;
int lastProc = 0;

int abs(int x) {
    return x > 0 ? x : -x;
}

int procToAngleMode0(int proc) {
    return (int)(proc * 18 / 10.f);
}

int procToAngleMode1(int proc) {
    return 180 - procToAngleMode1(proc);
}

int handler(int proc) {
    if (abs(proc - lastProc) > SWITCH_SPEED) {
        mode = 1 - mode;
    }
    lastProc = proc;

    if (mode == 0) {
        return procToAngleMode0(proc);
    } else {
        return procToAngleMode1(proc);
    }
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
   // TWIInit();

   // Configure Servo
   TCCR1A=(1<<COM1B1)|(1<<COM1A1)|(1<<WGM11);        // NON Inverted PWM
   TCCR1B=(1<<WGM13)|(1<<WGM12)|(1<<CS11);   //PRESCALER=64 MODE 14(FAST PWM)
   ICR1=39999;  // fPWM=50Hz (Period = 20ms Standard).
   DDRB|=(1<<PB2);   //PWM Pins as Out

   sei();

   /* Print hello  */
   printf("Hello world!\r\n");

   /* Pointer for EEReadData */
   // uint8_t* idp = (uint8_t *)malloc(8);

   int degree = 90;
   char input;   
   int interval = 10;


   rotate(degree, interval);


   while(1) {
      
      // EEWriteData((uint8_t)0x81);
      // EEReadData(idp);
      // printf("Product ID is %c\r\n", *idp);

      input = getchar();
      if (input == 'a') {
        degree -= 30;
      } else if (input == 'd') {
        degree += 30;
      } else if (input == 'w') {
         interval -= 50;
      } else if (input == 's') {
         interval += 50;
      }

      if (degree < 0) degree = 0;
      if (degree > 180) degree = 180;
      if (interval < 0) interval = 0;
      if (interval > 20) interval = 20;

      rotate(degree, interval);

      PORTB ^= 0x01;
   }
}
