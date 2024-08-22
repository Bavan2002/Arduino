#define F_CPU 16000000UL  // Define CPU clock speed
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <util/delay.h>

#define MOTOR1_PWM PD6 // OC0A (Output Compare pin for Timer0, Channel A)
#define MOTOR1_DIR PD2
#define MOTOR2_PWM PD5 // OC0B (Output Compare pin for Timer0, Channel B)
#define MOTOR2_DIR PD3

int pwm1 = 0, pwm2 = 0;

void uart_init(unsigned int ubrr) {
    // Set baud rate
    UBRR0H = (unsigned char)(ubrr>>8);
    UBRR0L = (unsigned char)ubrr;
    // Enable receiver and transmitter
    UCSR0B = (1<<RXEN0) | (1<<TXEN0);
    // Set frame format: 8 data bits, 1 stop bit
    UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);
}

unsigned char uart_receive(void) {
    // Wait for data to be received
    while (!(UCSR0A & (1<<RXC0)));
    // Get and return received data from buffer
    return UDR0;
}

void uart_print(char* str) {
    while (*str) {
        while (!(UCSR0A & (1<<UDRE0)));
        UDR0 = *str++;
    }
}

void uart_print_int(int value) {
    char buffer[10];
    itoa(value, buffer, 10);
    uart_print(buffer);
}

void parse_and_set_speeds(char* input) {
    // Parse the input string for two integers
    int speed1, speed2;
    if (sscanf(input, "%d %d", &speed1, &speed2) == 2) {
        // Print the received PWM values
        uart_print("Received PWM values: ");
        uart_print_int(speed1);
        uart_print(" ");
        uart_print_int(speed2);
        uart_print("\r\n");

        // Set motor speeds
        set_motor_speed(1, speed1);
        set_motor_speed(2, speed2);
    } else {
        uart_print("Error: Invalid input format.\r\n");
    }
}

void pwm_init() {
    // Set PWM pins as output
    DDRD |= (1<<MOTOR1_PWM) | (1<<MOTOR2_PWM);
    DDRD |= (1<<MOTOR1_DIR) | (1<<MOTOR2_DIR);

    // Set Fast PWM mode, non-inverted for Timer0
    TCCR0A = (1<<WGM00) | (1<<WGM01) | (1<<COM0A1) | (1<<COM0B1);
    // Set prescaler to 64 and start PWM for Timer0
    TCCR0B = (1<<CS01) | (1<<CS00);

    // Set Fast PWM mode, non-inverted for Timer1
    TCCR1A = (1<<WGM10) | (1<<COM1A1) | (1<<COM1B1);
    TCCR1B = (1<<WGM12) | (1<<CS11) | (1<<CS10); // Prescaler 64
}

void set_motor_speed(int motor, int speed) {
    if (speed >= 255){
      speed = 255;
    } else if (speed < -255){
      speed = -255;
    }
    if (motor == 1) {
        if (speed >= 0) {
            PORTD &= ~(1<<MOTOR1_DIR); // Set direction to forward
            OCR0A = speed; // Set PWM duty cycle
        } else {
            PORTD |= (1<<MOTOR1_DIR); // Set direction to backward
            OCR0A = 255 + speed; // Set PWM duty cycle
        }
    } else if (motor == 2) {
        if (speed >= 0) {
            PORTD &= ~(1<<MOTOR2_DIR); // Set direction to forward
            OCR0B = speed; // Set PWM duty cycle
        } else {
            PORTD |= (1<<MOTOR2_DIR); // Set direction to backward
            OCR0B = 255 + speed; // Set PWM duty cycle
        }
    }
}

int main(void){
    char input[20];
    int inputIndex = 0;
    char receivedChar;

    uart_init(103); // Initialize UART with baud rate 9600
    pwm_init(); // Initialize PWM

    sei(); // Enable global interrupts

    while(1){
        // Receive characters and build the input string
        receivedChar = uart_receive();
        if (receivedChar == '\n' || receivedChar == '\r') {
            input[inputIndex] = '\0'; // Null-terminate the string
            parse_and_set_speeds(input); // Parse and set speeds
            inputIndex = 0; // Reset input index for next input
        } else {
            input[inputIndex++] = receivedChar; // Add char to input string
        }
    }
}