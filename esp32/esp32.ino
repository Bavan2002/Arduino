#include <WiFi.h>
#include <WebServer.h>

// Replace these with your network credentials
const char* ssid = "Galaxy A32B46A";
const char* password = "aubz5725";


// Define motor control pins
#define MOTOR1_PWM 26 // Adjust these pins according to your ESP32 setup
#define MOTOR1_DIR 27
#define MOTOR2_PWM 25
#define MOTOR2_DIR 33

int pwm1 = 0, pwm2 = 0;

// Create a web server on port 80
WebServer server(80);

// Variable to store the last serial input
String lastSerialInput = "";

// Function to send the last received serial input
void handleSerialInputRequest() {
    server.send(200, "text/plain", lastSerialInput);
}

void uart_init(unsigned long baudrate) {
    Serial.begin(baudrate);
}

void pwm_init() {
    // Set motor direction pins as output
    pinMode(MOTOR1_DIR, OUTPUT);
    pinMode(MOTOR2_DIR, OUTPUT);

    // Set up PWM channels and frequency
    ledcSetup(0, 5000, 8); // Channel 0, 5kHz PWM, 8-bit resolution
    ledcSetup(1, 5000, 8); // Channel 1, 5kHz PWM, 8-bit resolution

    // Attach PWM channels to GPIO
    ledcAttachPin(MOTOR1_PWM, 0); // Attach MOTOR1_PWM to channel 0
    ledcAttachPin(MOTOR2_PWM, 1); // Attach MOTOR2_PWM to channel 1
}

void set_motor_speed(int motor, int speed) {
    if (speed >= 255){
      speed = 255;
    }else if(speed <= -255){
      speed = -255;
    }
    if (motor == 1) {
        if (speed >= 0) {
            digitalWrite(MOTOR1_DIR, LOW); // Set direction to forward
            ledcWrite(0, speed); // Set PWM duty cycle on channel 0
        } else {
            digitalWrite(MOTOR1_DIR, HIGH); // Set direction to backward
            ledcWrite(0, 255 + speed); // Set PWM duty cycle on channel 0
        }
    } else if (motor == 2) {
        if (speed >= 0) {
            digitalWrite(MOTOR2_DIR, LOW); // Set direction to forward
            ledcWrite(1, speed); // Set PWM duty cycle on channel 1
        } else {
            digitalWrite(MOTOR2_DIR, HIGH); // Set direction to backward
            ledcWrite(1, 255 + speed); // Set PWM duty cycle on channel 1
        }
    }
}

void parse_and_set_speeds(String input) {
    int speed1, speed2;
    if (sscanf(input.c_str(), "%d %d", &speed1, &speed2) == 2) {
        pwm1 = speed1;
        pwm2 = speed2;
        set_motor_speed(1, pwm1);
        set_motor_speed(2, pwm2);
    } else {
        Serial.println("Error: Invalid input format.");
    }
}

void setup() {
    // Initialize UART and PWM
    uart_init(115200);
    pwm_init();

    // Connect to Wi-Fi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("Connected to Wi-Fi");

    // Set up the web server endpoint
    // server.on("/serial_input", HTTP_GET, handleSerialInputRequest);
    
    // Start the web server
    server.begin();
}

void loop() {
    // Handle client requests
    // server.handleClient();

    // Check for incoming serial data
    if (Serial.available()) {
        lastSerialInput = Serial.readStringUntil('\n'); // Read input until newline
        parse_and_set_speeds(lastSerialInput); // Parse and set speeds based on serial input
    }
}

