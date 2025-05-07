#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
#include <Adafruit_NeoPixel.h>

// ===== Forward Declarations =====
void initializeSafeZone();
void checkGeofence();
float readTemperature();
void checkVibration();
void updateAndAlertSystem(float tempC);
void logSensorData(float tempC, int vibration);
void alarmBeep();
void connectToWiFi();
void checkAndReconnectWiFi();
void serialEvent();

// ===== Testing Mode Toggle =====
#define TESTING_MODE true  // Set to false for real GPS data

// ===== GPS Settings =====
HardwareSerial gpsSerial(1);  // UART1 (RX=GPIO7, TX=GPIO6)
TinyGPSPlus gps;
#define GPS_CHECK_INTERVAL 5000

// ===== Mock GPS Data =====
#if TESTING_MODE
double mockLat = 12.345678;  // Default coordinates
double mockLng = 98.765432;
bool mockMovingOutside = false;
#endif

// ===== Geofence Settings =====
#define FENCE_RADIUS 100  // Safe zone radius in meters
double FENCE_LAT = 0.0;
double FENCE_LNG = 0.0;
bool safeZoneInitialized = false;
bool geofenceBreached = false;
unsigned long lastGPSCheck = 0;
unsigned long lastMotorOffTime = 0;
#define MOTOR_DURATION 30000  // 30 seconds vibration when breached

// ===== WiFi and Telegram =====
const char* ssid = "iPhone";
const char* password = "Ayushi@123";
#define BOT_TOKEN "7531038127:AAE6q4MvTPZlIQkTle4Di5XpDg6LHSOzmCg"
#define CHAT_ID "7342444890"
WiFiClientSecure secured_client;
UniversalTelegramBot bot(BOT_TOKEN, secured_client);

// ===== Pin Definitions =====
#define MOTOR_PIN 5        // Vibration motor
#define VIBRATION_PIN 3    // SW-520D sensor
#define BUZZER_PIN 26      // Buzzer
#define LED_PIN 4          // NeoPixel
#define TMP36_PIN 2        // Temperature sensor

// ===== Constants =====
#define LED_COUNT 12
#define BRIGHTNESS 50
#define VIBRATION_THRESHOLD 100
#define COLD_TEMP 20.0
#define HOT_TEMP 34.0
#define MESSAGE_INTERVAL 15000
#define WIFI_CHECK_INTERVAL 30000
#define DATA_LOG_INTERVAL 5000  // Log data every 5 seconds

// ===== Global Variables =====
int vibrationCount = 0;
unsigned long lastCheckTime = 0;
unsigned long lastMessageTime = 0;
unsigned long lastWifiCheckTime = 0;
unsigned long lastDataLogTime = 0;
bool vibrationAlertSent = false;
bool temperatureAlertSent = false;
bool gpsAlertSent = false;
float lastLoggedTemp = 0;
int lastLoggedVibration = 0;
bool isPlotting = false;

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, 7, 6);

  // Initialize hardware
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);
  pinMode(VIBRATION_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  
  strip.begin();
  strip.setBrightness(BRIGHTNESS);
  strip.show();

  // Connect to WiFi
  connectToWiFi();
  secured_client.setInsecure();
  
  bot.sendMessage(CHAT_ID, "📍 Safety System Online (TEST MODE)", "");
}

void loop() {
  // GPS Data Processing
  #if !TESTING_MODE
    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
    }
  #endif

  // Main system checks
  if (millis() - lastGPSCheck >= GPS_CHECK_INTERVAL) {
    checkGeofence();
    lastGPSCheck = millis();
  }

  float tempC = readTemperature();
  checkVibration();
  
  if (millis() - lastCheckTime > 1000) {
    updateAndAlertSystem(tempC);
    lastCheckTime = millis();
  }

  // Data logging
  if (millis() - lastDataLogTime > DATA_LOG_INTERVAL) {
    logSensorData(tempC, vibrationCount);
    lastDataLogTime = millis();
  }

  // WiFi maintenance
  if (millis() - lastWifiCheckTime > WIFI_CHECK_INTERVAL) {
    checkAndReconnectWiFi();
    lastWifiCheckTime = millis();
  }

  // Command processing
  serialEvent();
}

// ===== Geofence Functions =====
void checkGeofence() {
  if (isPlotting) return; // Skip geofence checks while plotting
  
  double currentLat, currentLng;
  
  #if TESTING_MODE
    if (mockMovingOutside && !isPlotting) { // Only move if not plotting
      mockLat += 0.001; // ~100m north
      mockLng += 0.001; // ~100m east
    }
    currentLat = mockLat;
    currentLng = mockLng;
    Serial.print("[TEST] ");
    Serial.println(mockMovingOutside ? "Moving OUTSIDE safe zone" : "Inside safe zone");
  #else
    if (!safeZoneInitialized || !gps.location.isValid()) return;
    currentLat = gps.location.lat();
    currentLng = gps.location.lng();
  #endif

  double distance = TinyGPSPlus::distanceBetween(
    currentLat, currentLng, FENCE_LAT, FENCE_LNG
  );

  bool currentBreachStatus = (distance > FENCE_RADIUS);
  
  // Only trigger motor if breach status changed and we're not plotting
  if (currentBreachStatus != geofenceBreached && !isPlotting) {
    geofenceBreached = currentBreachStatus;
    if (geofenceBreached) {
      digitalWrite(MOTOR_PIN, HIGH);
      lastMotorOffTime = millis() + MOTOR_DURATION;
    } else {
      digitalWrite(MOTOR_PIN, LOW);
    }
  }
  
  // Turn off motor after duration (unless plotting)
  if (geofenceBreached && millis() > lastMotorOffTime && !isPlotting) {
    digitalWrite(MOTOR_PIN, LOW);
  }

  // Send alerts only when status changes and not plotting
  if (geofenceBreached && !gpsAlertSent && !isPlotting) {
    String msg = "🚨 ";
    msg += TESTING_MODE ? "*TEST ALERT* " : "";
    msg += "Person out of safe zone!\n📍 ";
    msg += "https://maps.google.com/?q=";
    msg += String(currentLat, 6) + "," + String(currentLng, 6);
    msg += "\n📏 Distance: " + String(distance) + "m";
    
    bot.sendMessage(CHAT_ID, msg, "Markdown");
    gpsAlertSent = true;
    alarmBeep();
  } 
  else if (!geofenceBreached && gpsAlertSent && !isPlotting) {
    String msg = "✅ Person returned to safe zone";
    bot.sendMessage(CHAT_ID, msg, "");
    gpsAlertSent = false;
  }
}

// ===== Sensor Functions =====
float readTemperature() {
  int rawValue = analogRead(TMP36_PIN);
  float voltage = rawValue * (3.3 / 4095.0);
  return (voltage - 0.5) * 100.0;
}

void checkVibration() {
  if (digitalRead(VIBRATION_PIN) == LOW) {
    vibrationCount++;
    delay(10); // Debounce
  }
}

// ===== Alert System =====
void updateAndAlertSystem(float tempC) {
  // LED Feedback
  uint32_t color;
  if (tempC < COLD_TEMP) color = strip.Color(0, 0, 255);      // Blue
  else if (tempC < HOT_TEMP) color = strip.Color(0, 255, 0);  // Green
  else color = strip.Color(255, 0, 0);                        // Red
  
  strip.fill(color, 0, LED_COUNT);
  strip.show();

  // Vibration Alert (only send if significant change)
  if (vibrationCount > VIBRATION_THRESHOLD && !vibrationAlertSent) {
    alarmBeep();
    String msg = "⚠️ High vibration detected: ";
    msg += String(vibrationCount) + " counts";
    bot.sendMessage(CHAT_ID, msg, "");
    vibrationAlertSent = true;
    lastMessageTime = millis();
  } 
  else if (vibrationCount <= VIBRATION_THRESHOLD/2 && vibrationAlertSent) {
    vibrationAlertSent = false;
  }

  // Temperature Alert (only send if significant change)
  if (tempC >= HOT_TEMP && !temperatureAlertSent) {
    String msg = "🔥 High temp alert: ";
    msg += String(tempC, 1) + "°C";
    bot.sendMessage(CHAT_ID, msg, "");
    temperatureAlertSent = true;
    lastMessageTime = millis();
    alarmBeep();
  } 
  else if (tempC < HOT_TEMP-2.0 && temperatureAlertSent) { // Hysteresis of 2°C
    temperatureAlertSent = false;
    String msg = "🌡️ Temperature normal: ";
    msg += String(tempC, 1) + "°C";
    bot.sendMessage(CHAT_ID, msg, "");
  }

  vibrationCount = 0;
}

// ===== Data Logging =====
void logSensorData(float tempC, int vibration) {
  // Store values for plotting
  lastLoggedTemp = tempC;
  lastLoggedVibration = vibration;

  // Print to Serial for plotting
  Serial.print("Temperature:");
  Serial.print(tempC);
  Serial.print(",Vibration:");
  Serial.println(vibration);

  #if TESTING_MODE
    // In testing mode, also print to serial
    Serial.print("[DATA] Temp: ");
    Serial.print(tempC);
    Serial.print("°C | Vib: ");
    Serial.println(vibration);
  #endif
}

// ===== Helper Functions =====
void alarmBeep() {
  for (int i = 0; i < 3; i++) {
    tone(BUZZER_PIN, 2000, 100);
    delay(200);
  }
}

void connectToWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected");
}

void checkAndReconnectWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect();
    connectToWiFi();
    secured_client.setInsecure();
  }
}

void serialEvent() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "start initialize safe zone") {
      initializeSafeZone();
    }
    else if (command == "plot") {
      isPlotting = true;
      Serial.println("Current sensor data:");
      Serial.print("Temperature:");
      Serial.print(lastLoggedTemp);
      Serial.print(",Vibration:");
      Serial.println(lastLoggedVibration);
      isPlotting = false; // Reset immediately after printing
    }
    #if TESTING_MODE
    else if (command == "simulate outside") {
      if (!isPlotting) { // Only process if not plotting
        mockMovingOutside = true;
        Serial.println("Simulating movement outside safe zone...");
      }
    }
    else if (command == "simulate inside") {
      if (!isPlotting) { // Only process if not plotting
        mockMovingOutside = false;
        mockLat = FENCE_LAT;
        mockLng = FENCE_LNG;
        Serial.println("Returned to safe zone");
      }
    }
    else if (command == "set temp") {
      if (!isPlotting) { // Only process if not plotting
        Serial.println("Enter temperature value:");
        while (!Serial.available());
        String tempStr = Serial.readStringUntil('\n');
        lastLoggedTemp = tempStr.toFloat();
        Serial.print("Set temp to: ");
        Serial.println(lastLoggedTemp);
      }
    }
    #endif
  }
}

void initializeSafeZone() {
  #if TESTING_MODE
    FENCE_LAT = mockLat;
    FENCE_LNG = mockLng;
    safeZoneInitialized = true;
    String msg = "🛡️ Safe Zone Set (TEST MODE)!\n";
    msg += String(FENCE_LAT, 6) + "," + String(FENCE_LNG, 6);
    bot.sendMessage(CHAT_ID, msg, "");
  #else
    unsigned long startTime = millis();
    while (millis() - startTime < 5000) {
      while (gpsSerial.available() > 0) {
        if (gps.encode(gpsSerial.read()) && gps.location.isValid()) {
          FENCE_LAT = gps.location.lat();
          FENCE_LNG = gps.location.lng();
          safeZoneInitialized = true;
          String msg = "🛡️ Safe Zone Set!\n";
          msg += String(FENCE_LAT, 6) + "," + String(FENCE_LNG, 6);
          bot.sendMessage(CHAT_ID, msg, "");
          return;
        }
      }
    }
    bot.sendMessage(CHAT_ID, "❌ Failed to set safe zone", "");
  #endif
}