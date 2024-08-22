#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

// Set the serial for communication with ROS2
#define SERIAL_ROS Serial

// Replace these with your network credentials
const char* ssid = "Galaxy A32B46A";
const char* password = "aubz5725";

StaticJsonDocument<200> doc;

// Create a web server on port 80
WebServer server(80);

// Variables to store velocities
float linear_velocity = 0;
float angular_velocity = 0;

// Function to send velocities
void sendVelocities() {
  String message = "Linear Velocity: " + String(linear_velocity) + ", Angular Velocity: " + String(angular_velocity);
  server.send(200, "text/plain", message);
}

void setup() {
  SERIAL_ROS.begin(57600); // ROS serial port, adjust pins as needed
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  // Define endpoint
  server.on("/velocities", HTTP_GET, sendVelocities);

  // Start server
  server.begin();
}

void loop() {
  server.handleClient();

  if (SERIAL_ROS.available()) {
    String data = SERIAL_ROS.readStringUntil('\n');
    DeserializationError error = deserializeJson(doc, data);
    if (!error) {
      linear_velocity = doc["linear_velocity"];
      angular_velocity = doc["angular_velocity"];
    }
  }

  delay(1000); // Small delay to stabilize loop
}
