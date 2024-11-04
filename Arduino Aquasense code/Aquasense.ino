#include "DHT.h"
#include <SoftwareSerial.h>

// Constants for DHT sensor
#define DHT_PIN 7          // Pin where DHT11/DHT22 is connected
#define DHT_TYPE DHT11     // Change to DHT22 if you're using DHT22

// Analog pin definitions for pH and turbidity sensors
#define PH_SENSOR_PIN A1
#define TURBIDITY_SENSOR_PIN A2

// Pins for Sound sensor and Buzzer
#define SOUND_SENSOR_PIN A0
#define BUZZER_PIN 8 // Define pin for the Piezo buzzer

// GSM Module pin definitions
SoftwareSerial mySerial(10, 11); // RX, TX for GSM module

// Create an instance of DHT
DHT dht(DHT_PIN, DHT_TYPE);

// Constants for sound level detection
const int sampleWindow = 50;   // Sample window width in milliseconds
unsigned int sample;

// Define threshold constants
const float pH_THRESHOLD_LOW = 7.0;      // Low pH threshold
const float pH_THRESHOLD_HIGH = 12.0;    // High pH threshold
const float TURBIDITY_THRESHOLD = 1.5;   // Turbidity threshold for alert
const int SOUND_THRESHOLD_DB = 150;      // Sound threshold for alert

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  mySerial.begin(9600);

  // Initialize DHT sensor
  dht.begin();

  // Set the sensor and buzzer pins
  pinMode(SOUND_SENSOR_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // Display initialization message for GSM
  Serial.println("Type:\n s) to send an SMS\n r) to receive an SMS\n c) to make a call");
  Serial.println("System Initializing...");
  delay(1000);

  // Initialize GSM module and check response
  if (initializeGSM()) {
    Serial.println("GSM module initialized successfully.");
  } else {
    Serial.println("GSM module initialization failed.");
  }
}

void loop() {
  // Check for commands from the Serial Monitor for GSM operations
  if (Serial.available() > 0) {
    char command = Serial.read();
    handleSerialCommand(command);
  }

  // Check for incoming data from GSM module
  if (mySerial.available()) {
    delay(1000);
    Serial.println(mySerial.readString());
  }

  // Sensor Readings and Alerts
  checkDHTSensor();
  checkPHSensor();
  checkTurbiditySensor();
  checkSoundLevel();

  // Delay between sensor readings
  delay(2000);  // Wait for 2 seconds before next reading
}

// Function to initialize GSM module
bool initializeGSM() {
  mySerial.println("AT");
  String response = readSerial();
  return response.indexOf("OK") != -1;
}

// Function to handle serial commands for GSM operations
void handleSerialCommand(char command) {
  switch (command) {
    case 's':
      SendMessage("Manual SMS Request");
      break;
    case 'r':
      ReceiveMessage();
      break;
    case 'c':
      CallNumber();
      break;
    default:
      Serial.println("Invalid command.");
  }
}

// Function to check DHT sensor readings
void checkDHTSensor() {
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Error: Failed to read from DHT sensor!");
  } else {
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.print(" %\tTemperature: ");
    Serial.print(temperature);
    Serial.println(" °C");
  }
}

// Function to check pH sensor readings
void checkPHSensor() {
  int pHAnalogValue = analogRead(PH_SENSOR_PIN);
  float pHVoltage = pHAnalogValue * (5.0 / 1023.0);
  float pH = 3.5 * pHVoltage; // Adjust based on sensor calibration
  Serial.print("pH Voltage: ");
  Serial.print(pHVoltage);
  Serial.print(" V\tpH Value: ");
  Serial.println(pH);

  // Check if pH is below 7
  if (pH < pH_THRESHOLD_LOW) {
    triggerAlert("Low pH level detected! pH below 7");
  }
  // Check if pH is above 12
  else if (pH > pH_THRESHOLD_HIGH) {
    triggerAlert("High pH level detected! pH above 12");
  }
}

// Function to check turbidity sensor readings
void checkTurbiditySensor() {
  int turbidityAnalogValue = analogRead(TURBIDITY_SENSOR_PIN);
  float turbidityVoltage = turbidityAnalogValue * (5.0 / 1023.0);
  Serial.print("Turbidity Voltage: ");
  Serial.print(turbidityVoltage);
  Serial.println(" V");

  if (turbidityVoltage < TURBIDITY_THRESHOLD) {
    triggerAlert("Low turbidity level detected!");
  }
}

// Function to check sound level readings
void checkSoundLevel() {
  int db = readSoundLevel();

  if (db >= SOUND_THRESHOLD_DB) {
    triggerAlert("High sound level detected!");
  }
}

// Function to read sound level
int readSoundLevel() {
  unsigned long startMillis = millis(); // Start of sample window
  float peakToPeak = 0;
  unsigned int signalMax = 0;
  unsigned int signalMin = 1024;

  while (millis() - startMillis < sampleWindow) {
    sample = analogRead(SOUND_SENSOR_PIN);
    if (sample < 1024) {
      if (sample > signalMax) signalMax = sample;
      if (sample < signalMin) signalMin = sample;
    }
  }

  peakToPeak = signalMax - signalMin;
  int db = map(peakToPeak, 20, 900, 30, 800);
  Serial.print("Loudness: ");
  Serial.print(db);
  Serial.println(" dB");
  return db;
}

// General function to trigger an alert with the buzzer and SMS message
void triggerAlert(String message) {
  digitalWrite(BUZZER_PIN, HIGH);
  Serial.println("Buzzer ON - " + message);
  SendMessage("Warning: " + message);
  delay(2000); // Allow the buzzer to sound for 2 seconds
  digitalWrite(BUZZER_PIN, LOW);
}

// Function to read serial data from GSM module
String readSerial() {
  delay(100);
  String response = "";
  while (mySerial.available()) {
    response += mySerial.readString();
  }
  return response;
}

// GSM Functions for Call, Send SMS, and Receive SMS
void CallNumber() {
  mySerial.println("ATD+639507898930;"); // Replace with your phone number
  Serial.println("Dialing...");
  Serial.println(readSerial());
  delay(20000); // Wait for 20 seconds
  mySerial.println("ATH"); // Hang up the call
  delay(1000);
  Serial.println("Call Ended");
  Serial.println(readSerial());
}

void SendMessage(String message) {
  mySerial.println("AT+CMGF=1");     // Text mode
  Serial.println(readSerial());
  mySerial.println("AT+CMGS=\"+639619883000\""); // Phone number
  Serial.println(readSerial());
  mySerial.println(message); // Send dynamic message
  mySerial.write(26); // ASCII for CTRL+Z to send the message
  Serial.println("Message Sent!");
  Serial.println(readSerial());
}

void ReceiveMessage() {
  Serial.println("Reading SMS...");
  mySerial.println("AT+CMGF=1");    // Text mode
  Serial.println(readSerial());
  mySerial.println("AT+CNMI=1,2,0,0,0"); // Auto-receive SMS
  Serial.println(readSerial());
}
