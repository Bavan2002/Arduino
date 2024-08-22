#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>

#define I2C_FREQ 100000L
#define AS5600_ADDRESS 0x36 // Default device I2C address
#define SCL  PINC5  // Define I2C pins
#define SDA  PINC4

#define MOTOR1_PWM PD6 // OC0A (Output Compare pin for Timer0, Channel A)
#define MOTOR1_DIR PD2
#define MOTOR2_PWM PD5 // OC0B (Output Compare pin for Timer0, Channel B)
#define MOTOR2_DIR PD3

#define MAX_PWM 255
#define PID_RATE 3 // Hz
#define PID_INTERVAL 1000 / PID_RATE
#define AUTO_STOP_INTERVAL 2000 // Time in milliseconds before stopping the motors if no command is received

volatile uint8_t update_flag = 0;  // Flag to signal main loop

// Encoder variables
volatile uint8_t byte1 = 0;
volatile uint8_t byte2 = 0;
volatile uint32_t byte3 = 0;
volatile uint32_t byte4 = 0;
volatile uint8_t q_0 = 0;
volatile uint8_t q_1 = 0;
volatile uint8_t preQ_0 = 0;
volatile uint8_t preQ_1 = 0;
volatile float angle_0;
volatile float angle_1;
volatile long targetTicksPerFrame1 = 0;
volatile long targetTicksPerFrame2 = 0;
volatile unsigned long timer2_millis = 0;


volatile unsigned long nextPID = PID_INTERVAL;
volatile unsigned long lastMotorCommand = 0; // Timer to track last motor command
volatile uint8_t moving = 0; // Flag to indicate if the motors are moving

// PID parameters and variables
int Kp = 8;
int Kd = 2;
int Ki = 0;
int Ko = 1;
int pid;

// int Kp = 5;
// int Kd = 16;
// int Ki = 0;
// int Ko = 1;
volatile long encoder1 = 0, encoder2 = 0;
volatile int leftPID_Output = 0, rightPID_Output = 0;
volatile int left_prev_input = 0, right_prev_input = 0;
volatile int left_ITerm = 0, right_ITerm = 0;
volatile int left_prev_encoder = 0, right_prev_encoder = 0;

// Global variables
volatile uint16_t adc_value_0;
volatile uint16_t adc_value_1;

// Function to initialize the ADC
// void ADC_init()
// {
//     // Select AVCC as the reference voltage with external capacitor at AREF pin
//     ADMUX |= (1 << REFS0);

//     // Enable ADC, set the prescaler to 128 (16MHz / 128 = 125kHz ADC clock)
//     ADCSRA |= (1 << ADEN);   // Enable ADC
//     ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Set prescaler to 128
// }

// // Function to read from the specified ADC channel (0 = A0, 1 = A1, etc.)
// uint16_t ADC_read(uint8_t channel)
// {
//     // Set the corresponding channel by updating the MUX bits
//     ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);  // Clear the previous channel bits and set the new channel

//     // Start the ADC conversion
//     ADCSRA |= (1 << ADSC);

//     // Wait for the conversion to complete
//     while(ADCSRA & (1 << ADSC));

//     // Return the ADC value
//     return ADC;
// }


void ADC_init() {
    ADMUX |= (1 << REFS1) | (1 << REFS0);  // Use the internal 1.1V reference
    ADCSRA |= (1 << ADEN);  // Enable ADC
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);  // Prescaler = 128
}

uint16_t ADC_read(uint8_t channel) {
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);  // Select the ADC channel
    ADCSRA |= (1 << ADSC);  // Start conversion
    while (ADCSRA & (1 << ADSC));  // Wait for conversion to complete

    return ADC;
}


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
    while (!(UCSR0A & (1 << RXC0))); // Wait for data to be received
    return UDR0; // Get and return received data from buffer
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

void startI2C(uint32_t address) {    // Start I2C
    TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT); // Enable I2C and generate START condition
    while (!(TWCR & (1 << TWINT))); // Wait until I2C finish its current work
    TWDR = address;  // Write SLA+W in I2C data register
    TWCR = (1 << TWEN) | (1 << TWINT); // Enable I2C and clear interrupt flag
    while (!(TWCR & (1 << TWINT))); // Wait until I2C finish its current work
}

void send_encoder_turns() {
    // Send the number of turns to serial output
    char buffer[20];
    sprintf(buffer, "%ld %ld\n", byte3, byte4);
    USART_sendString(buffer);
}

void setup_timer2_interrupt() {
    TCCR2A = (1 << WGM21); // Set CTC mode (WGM21 = 1, WGM20 = 0)
    TCCR2B = (1 << CS22); // Set prescaler to 64 (CS22 = 1, CS21 = 0, CS20 = 0)
    OCR2A = 249; // Set compare match value for 1ms interrupt
    TIMSK2 = (1 << OCIE2A); // Enable Timer2 compare interrupt
}

void uart_print_ulong(unsigned long value) {
    char buffer[11];  // Buffer to hold the string representation of the number
    ultoa(value, buffer, 10);  // Convert the unsigned long to a string
    uart_print(buffer);  // Print the string
}

unsigned long millis_custom() {
    unsigned long millis_return;

    // Disable interrupts while reading the value to prevent data corruption
    cli();
    millis_return = timer2_millis;
    sei();

    return millis_return;
}



ISR(TIMER2_COMPA_vect) {
    timer2_millis++;
    update_encoders(); // Call update_encoder() function during Timer2 interrupt
    if (millis_custom() > nextPID) {
        if (pid){updatePID();}
        nextPID += PID_INTERVAL;
    }
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




void parse_and_set_counts(char* input) {
    // Parse the input string for two integers
    int count1, count2;
    pid = 1;

    if (sscanf(input, "%d %d", &count1, &count2) == 2) {
        lastMotorCommand = millis_custom(); // Reset the auto-stop timer
        if (count1 == 0 && count2 == 0) {
            set_motor_speed(1, 0);
            set_motor_speed(2, 0);
            resetPID();
            moving = 0;
        } else {
            moving = 1;
            targetTicksPerFrame1 = count1;
            targetTicksPerFrame2 = count2;
            // uart_print("PID Targets Set: ");
            // uart_print_int(count1);
            // uart_print(" ");
            // uart_print_int(count2);
            // uart_print("\r\n");
        }
    } else {
        uart_print("Error: Invalid input format.\r\n");
    }
}

void pwm_init() {
    DDRD |= (1 << MOTOR1_PWM) | (1 << MOTOR2_PWM); // Set PWM pins as output
    DDRD |= (1 << MOTOR1_DIR) | (1 << MOTOR2_DIR);

    TCCR0A = (1 << WGM00) | (1 << WGM01) | (1 << COM0A1) | (1 << COM0B1); // Set Fast PWM mode, non-inverted for Timer0
    TCCR0B = (1 << CS01) | (1 << CS00); // Set prescaler to 64 and start PWM for Timer0

    TCCR1A = (1 << WGM10) | (1 << COM1A1) | (1 << COM1B1); // Set Fast PWM mode, non-inverted for Timer1
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

// uint8_t getAngle(uint8_t reg) {
//     uint8_t byte;

//     TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT); // Start I2C transmission to write the register address
//     while (!(TWCR & (1 << TWINT)));

//     TWDR = (AS5600_ADDRESS << 1) | 0; // Address + Write bit
//     TWCR = (1 << TWEN) | (1 << TWINT);
//     while (!(TWCR & (1 << TWINT)));

//     TWDR = reg;
//     TWCR = (1 << TWEN) | (1 << TWINT);
//     while (!(TWCR & (1 << TWINT)));

//     TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT); // Start I2C transmission to read the data
//     while (!(TWCR & (1 << TWINT)));

//     TWDR = (AS5600_ADDRESS << 1) | 1; // Address + Read bit
//     TWCR = (1 << TWEN) | (1 << TWINT);
//     while (!(TWCR & (1 << TWINT)));

//     TWCR = (1 << TWEN) | (1 << TWINT); // Read byte
//     while (!(TWCR & (1 << TWINT)));
//     byte = TWDR;

//     TWCR = (1 << TWSTO) | (1 << TWINT) | (1 << TWEN); // Stop I2C transmission
//     while (TWCR & (1 << TWSTO));

//     return byte;
// }

void update_single_encoder(uint8_t channel, uint16_t* adc_value, float* angle, uint8_t* q, uint8_t* preQ, volatile uint32_t* turn_count)
{
    // Read the ADC value from the specified channel (0 for A0, 1 for A1)
    *adc_value = ADC_read(channel);

    // Adjusted angle calculation based on maximum ADC value 1024
    *angle = (*adc_value / 1024.0) * 360.0;


    // Determine the quadrant based on the angle
    if (0 <= *angle && *angle <= 90) *q = 1;
    else if (90 < *angle && *angle <= 180) *q = 2;
    else if (180 < *angle && *angle <= 270) *q = 3;
    else if (270 < *angle && *angle < 360) *q = 4;

    // Check for quadrant transition to update turn count
    if (*q != *preQ) {
        if (*q == 1 && *preQ == 4) (*turn_count)++;  // Increase turns count
        if (*q == 4 && *preQ == 1) (*turn_count)--;  // Decrease turns count
        *preQ = *q;  // Update previous quadrant
    }
}

void update_encoders()
{


        update_single_encoder(0, &adc_value_0, &angle_0, &q_0, &preQ_0, &byte3);
        update_single_encoder(1, &adc_value_1, &angle_1, &q_1, &preQ_1, &byte4);
}



void resetPID() {
    leftPID_Output = 0;
    rightPID_Output = 0;
    left_prev_input = 0;
    right_prev_input = 0;
    left_ITerm = 0;
    right_ITerm = 0;

    encoder1 = byte3;
    encoder2 = byte4;
    left_prev_encoder = encoder1;
    right_prev_encoder = encoder2;

    targetTicksPerFrame1 = 0;
    targetTicksPerFrame2 = 0;
}

void doPID(int *pid_output, long encoder_count, int target_ticks_per_frame, int *prev_encoder, int *prev_input, int *ITerm) {
    long Perror;
    long output;
    int input = encoder_count - *prev_encoder;

    Perror = target_ticks_per_frame - input;

    if (abs(Perror) < 2) {
        Perror = 0;
    }
    output = (Kp * Perror - Kd * (input - *prev_input) + *ITerm) / Ko;
    *prev_encoder = encoder_count;


    output += *pid_output;
    if (output >= MAX_PWM)
        output = MAX_PWM;
    else if (output <= -MAX_PWM)
        output = -MAX_PWM;
    else
        *ITerm += Ki * Perror;

    *pid_output = output;
    *prev_input = input;
}

// void updatePID() {
//     if (moving) {
//         encoder1 = byte3;
//         encoder2 = byte3;
//         doPID(&leftPID_Output, encoder1, targetTicksPerFrame1, &left_prev_encoder, &left_prev_input, &left_ITerm);
//         doPID(&rightPID_Output, encoder2, targetTicksPerFrame2, &right_prev_encoder,&right_prev_input, &right_ITerm);

//         set_motor_speed(1, leftPID_Output);
//         set_motor_speed(2, rightPID_Output);
//     }
//     // Check for auto-stop condition
//     // if (millis() - lastMotorCommand > AUTO_STOP_INTERVAL) {
//     //     set_motor_speed(1, 0);
//     //     set_motor_speed(2, 0);
//     //     moving = 0;
//     // }
// }

void updatePID() {
        // Update encoder values (this should reflect the actual current position)
        encoder1 = byte3;
        encoder2 = byte4;

        // // Continuously update target position to move forward
        // static long targetPosition1 = 0;
        // static long targetPosition2 = 0;

        // targetPosition1 += targetTicksPerFrame1;
        // targetPosition2 += targetTicksPerFrame2;

        // Calculate PID based on the difference between current and target positions
        doPID(&leftPID_Output, encoder1, targetTicksPerFrame1, &left_prev_encoder, &left_prev_input, &left_ITerm);
        doPID(&rightPID_Output, encoder2, targetTicksPerFrame2, &right_prev_encoder, &right_prev_input, &right_ITerm);

        // Set the motor speeds based on PID outputs
        set_motor_speed(1, leftPID_Output);
        set_motor_speed(2, rightPID_Output);
     if (!moving){ 
        if(left_prev_encoder != 0 || right_prev_encoder != 0){
        resetPID();
        }
     }
}

int main(void) {
    char input[20];
    int inputIndex = 0;
    char receivedChar;

    ADC_init();

    USART_init(103); // Initialize USART with baud rate 9600
    pwm_init(); // Initialize PWM

    DDRC |= (1 << SDA) | (1 << SCL);   // Set SCL and SDA as output
    PORTC |= (1 << SDA) | (1 << SCL);  // Set SCL and SDA high

    TWSR = 0x00; // Set pre-scaler to 1
    TWBR = ((F_CPU / I2C_FREQ) - 16) / 2;   // Initialize I2C frequency

   // ADCSRA = 0;  // Disable ADC
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
             else if (input[0] == 'm') {
                parse_and_set_counts(input + 1); // Parse and set counts

            } 
            else {
                parse_and_set_speeds(input); // Parse and set speeds
                resetPID();
                pid = 0;
                 //             uart_print_ulong(millis_custom());
                // uart_print(" ");
                // uart_print_int(targetTicksPerFrame1);
                // uart_print("\r\n");
            }

            inputIndex = 0; // Reset input index for next input
        } else {
            input[inputIndex++] = receivedChar; // Add char to input string
        }
    }
}

