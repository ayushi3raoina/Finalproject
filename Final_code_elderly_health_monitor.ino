#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>

// Wi-Fi credentials
#define WIFI_SSID "TLU"
#define WIFI_PASSWORD ""

// Firebase credentials
#define API_KEY "AIzaSyDhSMjI4YasYswfwdxNwKP0BrlBXglplVU"
#define DATABASE_URL "https://esp32-final-252a1-default-rtdb.europe-west1.firebasedatabase.app"
#define USER_EMAIL "ayushi@tlu.ee"
#define USER_PASSWORD "Ayushi123"

// Firebase objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// Pin Definitions
const int sdaPin = 21;          // I2C SDA
const int sclPin = 20;          // I2C SCL
const int buzzerPin = 26;       // Buzzer
const int vibrationMotorPin = 3; // Vibration motor
const int tempSensorPin = 5;   // TMP36 analog input
const int neoPixelPin = 6;      // NeoPixel data
const int accelAddress = 0x68;  // MPU6050 address

// NeoPixel Setup - Changed to 8 pixels as example
const int numPixels = 8;  // Change this to your actual number of pixels
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(numPixels, neoPixelPin, NEO_GRB + NEO_KHZ800);

// Thresholds
const float standingThreshold = 0.85;    // G-force for standing
const float bendingThreshold = 0.55;     // G-force for bending
const int requiredConsecutiveFalls = 20; // Fall detection samples
const float tempLowThreshold = 20.0;     // Below this = cold (blue)
const float tempHighThreshold = 34.0;    // Changed to 32°C for fever (red)
const int printInterval = 500;           // Serial print interval

// State Tracking
enum Location { INDOOR, OUTDOOR };
Location currentLocation = INDOOR;
String postureState = "STANDING";
String tempState = "NORMAL";
unsigned long lastPrintTime = 0;

void setup() {
  Serial.begin(115200);
  
  // Initialize I2C
  Wire.begin(sdaPin, sclPin);
  
  // Initialize pins
  pinMode(buzzerPin, OUTPUT);
  pinMode(vibrationMotorPin, OUTPUT);
  pinMode(tempSensorPin, INPUT);

  // Wifi start
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi connected!");

  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  
  // Initialize Accelerometer
  Wire.beginTransmission(accelAddress);
  Wire.write(0x6B); Wire.write(0); // Wake up
  Wire.endTransmission();
  Wire.beginTransmission(accelAddress);
  Wire.write(0x1C); Wire.write(0x00); // ±2g range
  Wire.endTransmission();
  
  // Initialize NeoPixel
  pixels.begin();
  pixels.show(); // Initialize to off
  
  Serial.println("System Ready - Send 'inside' or 'outside'");
}

void loop() {
  checkLocationCommands();
  updatePostureDetection();
  updateTemperature();
  updateVibration();
  updateNeoPixel();
  showAllStatuses();
  delay(50); // 20Hz sampling
}

void checkLocationCommands() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.equalsIgnoreCase("inside")) currentLocation = INDOOR;
    else if (input.equalsIgnoreCase("outside")) currentLocation = OUTDOOR;
  }
}

void updatePostureDetection() {
  static int fallCounter = 0;
  float x, y, z;
  
  // Read accelerometer
  Wire.beginTransmission(accelAddress);
  Wire.write(0x3B); // ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(accelAddress, 6, true);
  x = (Wire.read()<<8|Wire.read())/16384.0;
  y = (Wire.read()<<8|Wire.read())/16384.0;
  z = (Wire.read()<<8|Wire.read())/16384.0;
  
  float verticalG = abs(z);

  if (verticalG > standingThreshold) {
    postureState = "STANDING";
    fallCounter = 0;
    noTone(buzzerPin);
  } 
  else if (verticalG > bendingThreshold) {
    postureState = "BENDING"; 
    fallCounter = 0;
    noTone(buzzerPin);
  }
  else {
    fallCounter++;
    if (fallCounter >= requiredConsecutiveFalls) {
      postureState = "FALL";
      tone(buzzerPin, 3000);
    }
  }
}

void updateTemperature() {
  // Read TMP36 (10mV/°C, 500mV offset)
  int rawValue = analogRead(tempSensorPin);
  float voltage = rawValue * (3.3 / 4095.0); // ESP32 ADC resolution
  float tempC = (voltage - 0.5) * 100; // TMP36 conversion formula
  
  if (tempC < tempLowThreshold) {
    tempState = "COLD";
  } 
  else if (tempC <= tempHighThreshold) {
    tempState = "NORMAL";
  }
  else {
    tempState = "FEVER";
  }
}

void updateNeoPixel() {
  uint32_t color;
  
  if (tempState == "COLD") {
    color = pixels.Color(0, 0, 255); // Blue
  }
  else if (tempState == "NORMAL") {
    color = pixels.Color(0, 255, 0); // Green
  }
  else {
    color = pixels.Color(255, 0, 0); // Red
  }
  
  // Light up all pixels
  for(int i=0; i<numPixels; i++) {
    pixels.setPixelColor(i, color);
  }
  pixels.show();
}

void updateVibration() {
  digitalWrite(vibrationMotorPin, (currentLocation == OUTDOOR) ? HIGH : LOW);
}

void showAllStatuses() {
  if (millis() - lastPrintTime >= printInterval) {
    // Format: Posture | Location | Temperature
    Serial.print(postureState);
    Serial.print((postureState == "FALL") ? " DETECTED | " : "         | ");
    Serial.print(currentLocation == INDOOR ? "Inside safe zone | " : "Outside - UNSAFE | ");
    Serial.print("Temp: ");
    Serial.println(tempState);
    
    lastPrintTime = millis();

    sendToDbAllStatuses();
  }
}

void sendToDbAllStatuses() {
     Serial.println("Sent data to firebase");
    Firebase.RTDB.setString(&fbdo, "/status/posture", postureState);
    Firebase.RTDB.setBool(&fbdo, "/status/safezone", currentLocation == INDOOR);
    Firebase.RTDB.setString(&fbdo, "/status/temperature", tempState);
}

