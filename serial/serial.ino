#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// Replace with your network credentials
const char* ssid = "Galaxy A32B46A";
const char* password = "aubz5725";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Serial data buffer
String serialData;

void setup() {
  Serial.begin(57600); // Start serial communication at 115200 baud rate

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    // Serial.println("Connecting to WiFi...");
  }

  // Print ESP32 Local IP Address
  // Serial.println(WiFi.localIP());

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", serialData);
  });

  // Start server
  server.begin();
}

void loop() {
  if (Serial.available()) {
    // Read the incoming data from Serial2
    serialData = Serial.readString();
  }
}
