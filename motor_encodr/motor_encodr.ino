#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>

#define I2C_FREQ 100000L
#define AS5600_ADDRESS 0x36 // Default device I2C address
#define SCL  PINC5  //Define I2C pins
#define SDA  PINC4

#define MOTOR1_PWM PD6 // OC0A (Output Compare pin for Timer0, Channel A)
#define MOTOR1_DIR PD2
#define MOTOR2_PWM PD5 // OC0B (Output Compare pin for Timer0, Channel B)
#define MOTOR2_DIR PD3


volatile uint8_t update_flag = 0;  // Flag to signal main loop

// Encoder variables
volatile uint8_t byte1 = 0;
volatile uint8_t byte2 = 0;
volatile uint32_t byte3 = 0;
volatile uint8_t q = 0;
volatile uint8_t preQ = 0;
volatile float angle;

void USART_init(uint16_t ubrr) {
    UBRR0H = (ubrr >> 8);
    UBRR0L = ubrr;
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);  // Enable transmitter and receiver
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);  // Set frame format: 8 data bits, 1 stop bit
}

void USART_send(char data) {
    while (!(UCSR0A & (1 << UDRE0)));  // Wait until buffer is empty
    UDR0 = data;  // Put data into buffer and send
}

void USART_sendString(const char* str) {
    while (*str) {
        USART_send(*str++);
    }
}


unsigned char uart_receive(void) {
    // Wait for data to be received
    while (!(UCSR0A & (1 << RXC0)));
    // Get and return received data from buffer
    return UDR0;
}

void uart_print(char* str) {
    while (*str) {
        while (!(UCSR0A & (1 << UDRE0)));
        UDR0 = *str++;
    }
}

void uart_print_int(int value) {
    char buffer[10];
    itoa(value, buffer, 10);
    uart_print(buffer);
}

void startI2C(uint32_t address) {    //start I2C
    TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT); //Enable I2C and generate START condition
    while (!(TWCR & (1 << TWINT))); //Wait until I2C finish its current work
    TWDR = address;  //Write SLA+W in I2C data register
    TWCR = (1 << TWEN) | (1 << TWINT); // Enable I2C and clear interrupt flag
    while (!(TWCR & (1 << TWINT))); // Wait until I2C finish its current work
}

void send_encoder_turns() {
    // Send the number of turns to serial output
    char buffer[20];
    sprintf(buffer, "Encoder Turns: %ld\n", byte3);
    USART_sendString(buffer);
}


void setup_timer2_interrupt() {
    // Set CTC mode (WGM21 = 1, WGM20 = 0)
    TCCR2A = (1 << WGM21);

    // Set prescaler to 64 (CS22 = 1, CS21 = 0, CS20 = 0)
    TCCR2B = (1 << CS22);

    // Set compare match value for 1ms interrupt
    OCR2A = 249;

    // Enable Timer2 compare interrupt
    TIMSK2 = (1 << OCIE2A);
}

ISR(TIMER2_COMPA_vect) {
    update_encoder(); // Call update_encoder() function during Timer1 interrupt
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
    DDRD |= (1 << MOTOR1_PWM) | (1 << MOTOR2_PWM);
    DDRD |= (1 << MOTOR1_DIR) | (1 << MOTOR2_DIR);

    // Set Fast PWM mode, non-inverted for Timer0
    TCCR0A = (1 << WGM00) | (1 << WGM01) | (1 << COM0A1) | (1 << COM0B1);
    // Set prescaler to 64 and start PWM for Timer0
    TCCR0B = (1 << CS01) | (1 << CS00);

    // Set Fast PWM mode, non-inverted for Timer1
    TCCR1A = (1 << WGM10) | (1 << COM1A1) | (1 << COM1B1);
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10); // Prescaler 64
}

void set_motor_speed(int motor, int speed) {
    if (speed >= 255) {
        speed = 255;
    } else if (speed < -255) {
        speed = -255;
    }
    if (motor == 1) {
        if (speed >= 0) {
            PORTD &= ~(1 << MOTOR1_DIR); // Set direction to forward
            OCR0A = speed; // Set PWM duty cycle
        } else {
            PORTD |= (1 << MOTOR1_DIR); // Set direction to backward
            OCR0A = 255 + speed; // Set PWM duty cycle
        }
    } else if (motor == 2) {
        if (speed >= 0) {
            PORTD &= ~(1 << MOTOR2_DIR); // Set direction to forward
            OCR0B = speed; // Set PWM duty cycle
        } else {
            PORTD |= (1 << MOTOR2_DIR); // Set direction to backward
            OCR0B = 255 + speed; // Set PWM duty cycle
        }
    }
}

uint8_t getAngle(uint8_t reg) {
    uint8_t byte;

    // Start I2C transmission to write the register address
    TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));

    TWDR = (AS5600_ADDRESS << 1) | 0; // Address + Write bit
    TWCR = (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));

    TWDR = reg;
    TWCR = (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));

    // Start I2C transmission to read the data
    TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));

    TWDR = (AS5600_ADDRESS << 1) | 1; // Address + Read bit
    TWCR = (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));

    // Read byte
    TWCR = (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));
    byte = TWDR;

    // Stop I2C transmission
    TWCR = (1 << TWSTO) | (1 << TWINT) | (1 << TWEN);
    while (TWCR & (1 << TWSTO));

    return byte;
}

void update_encoder() {
    byte1 = getAngle(0x0D); // Get high byte
    byte2 = getAngle(0x0C); // Get low byte

    uint16_t int1 = (int)byte1;
    uint16_t int2 = (int)byte2;

    float raw_angle = (int2 << 8) | int1;
    angle = raw_angle * 360 / 4096;

    // Update the quadrant
    if (0 <= angle && angle <= 90) q = 1;
    else if (90 < angle && angle <= 180) q = 2;
    else if (180 < angle && angle <= 270) q = 3;
    else if (270 < angle && angle < 360) q = 4;

    if (q != preQ) {
        if (q == 1 && preQ == 4) byte3++; // Increase turns count
        if (q == 4 && preQ == 1) byte3--; // Decrease turns count
        preQ = q;
    }

    // Debugging
    // char buffer[50];
    // sprintf(buffer, "Raw Angle: %d, Angle: %f, Turns: %ld\n", (int)raw_angle, angle, byte3);
    // USART_sendString(buffer);
}

int main(void) {
    char input[20];
    int inputIndex = 0;
    char receivedChar;

    USART_init(103); // Initialize USART with baud rate 9600
    pwm_init(); // Initialize PWM

    DDRC |= (1 << SDA) | (1 << SCL);   //set SCL and SDA as output
    PORTC |= (1 << SDA) | (1 << SCL);  //set SCL and SDA high

    TWSR = 0x00; // Set pre-scaler to 1
    TWBR = ((F_CPU / I2C_FREQ) - 16) / 2;   //initialize i2c frequency

    ADCSRA = 0;  //disable ADC
    setup_timer2_interrupt();
    sei(); // Enable global interrupts

    while (1) {
        // Receive characters and build the input string
        receivedChar = uart_receive();
        if (receivedChar == '\n' || receivedChar == '\r') {
            input[inputIndex] = '\0'; // Null-terminate the string

            if (input[0] == 'e') {
                send_encoder_turns(); // Send encoder turns count
            } 
            else {
                parse_and_set_speeds(input); // Parse and set speeds
            }

            inputIndex = 0; // Reset input index for next input
        } else {
            input[inputIndex++] = receivedChar; // Add char to input string
        }
    }
}

