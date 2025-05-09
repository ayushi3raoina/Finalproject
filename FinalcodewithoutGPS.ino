#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
#include <Adafruit_NeoPixel.h>
#include <HTTPClient.h>

// ===== Configuration Constants =====
#define TESTING_MODE true
#define GPS_CHECK_INTERVAL 5000
#define FENCE_RADIUS 100
#define MOTOR_DURATION 30000
#define LED_COUNT 12
#define BRIGHTNESS 50
#define VIBRATION_THRESHOLD 100
#define COLD_TEMP 20.0
#define HOT_TEMP 34.0
#define MESSAGE_INTERVAL 15000
#define WIFI_CHECK_INTERVAL 30000
#define DATA_LOG_INTERVAL 5000
#define ALERT_INTERVAL 30000

// ===== Pin Definitions =====
#define MOTOR_PIN 5
#define VIBRATION_PIN 3
#define BUZZER_PIN 26
#define LED_PIN 4
#define TMP36_PIN 2

// ===== WiFi and Telegram =====
const char* ssid = "iPhone";
const char* password = "Ayushi@123";
#define BOT_TOKEN "7531038127:AAE6q4MvTPZlIQkTle4Di5XpDg6LHSOzmCg"
#define CHAT_ID "7342444890"

// ===== Google Sheets =====
const char* googleScriptUrl = "https://script.google.com/macros/s/AKfycbyyd1zGwH2tJb0TUwfSC3vQ1fj9o2vbZlEdbBicvcbolXYeNzS5KqqWPPG0REKf3JlMUQ/exec";

// ===== Global Variables =====
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;
WiFiClientSecure secured_client;
UniversalTelegramBot bot(BOT_TOKEN, secured_client);
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

#if TESTING_MODE
double mockLat = 12.345678;
double mockLng = 98.765432;
bool mockMovingOutside = false;
#endif

double FENCE_LAT = 0.0;
double FENCE_LNG = 0.0;
bool safeZoneInitialized = false;
bool geofenceBreached = false;
bool isPlotting = false;
bool vibrationAlertSent = false;
bool temperatureAlertSent = false;
bool gpsAlertSent = false;
bool lastStatusNormal = true;

int vibrationCount = 0;
float lastLoggedTemp = 0;
int lastLoggedVibration = 0;

unsigned long lastGPSCheck = 0;
unsigned long lastMotorOffTime = 0;
unsigned long lastCheckTime = 0;
unsigned long lastMessageTime = 0;
unsigned long lastWifiCheckTime = 0;
unsigned long lastDataLogTime = 0;
unsigned long lastAlertTime = 0;

void connectToWiFi();
void checkGeofence();
float readTemperature();
void checkVibration();
void updateAndAlertSystem(float tempC);
void logSensorData(float tempC, int vibration);
void alarmBeep();
void vibrationAlarm();
void checkAndReconnectWiFi();
void serialEvent();
void sendAlert(String message);
void sendToGoogleSheet(float tempC, int vibration, bool geofenceStatus, double lat, double lng);
void initializeSafeZone();

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, 7, 6);

  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);
  pinMode(VIBRATION_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  
  strip.begin();
  strip.setBrightness(BRIGHTNESS);
  strip.show();

  connectToWiFi();
  secured_client.setInsecure();
  bot.sendMessage(CHAT_ID, "📍 Safety System Online (TEST MODE)", "");
}

void loop() {
  #if !TESTING_MODE
    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
    }
  #endif

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

  if (millis() - lastDataLogTime > DATA_LOG_INTERVAL) {
    logSensorData(tempC, vibrationCount);
    lastDataLogTime = millis();
  }

  if (millis() - lastWifiCheckTime > WIFI_CHECK_INTERVAL) {
    checkAndReconnectWiFi();
    lastWifiCheckTime = millis();
  }

  serialEvent();
}

void connectToWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected");
}

void checkGeofence() {
  if (isPlotting) return;
  
  double currentLat, currentLng;
  
  #if TESTING_MODE
    if (mockMovingOutside && !isPlotting) {
      mockLat += 0.001;
      mockLng += 0.001;
    }
    currentLat = mockLat;
    currentLng = mockLng;
  #else
    if (!safeZoneInitialized || !gps.location.isValid()) return;
    currentLat = gps.location.lat();
    currentLng = gps.location.lng();
  #endif

  double distance = TinyGPSPlus::distanceBetween(currentLat, currentLng, FENCE_LAT, FENCE_LNG);
  geofenceBreached = (distance > FENCE_RADIUS);
  
  if (geofenceBreached && millis() > lastMotorOffTime && !isPlotting) {
    digitalWrite(MOTOR_PIN, LOW);
  }
}

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

void alarmBeep() {
  for (int i = 0; i < 3; i++) {
    tone(BUZZER_PIN, 3000, 100); // High-pitched beep (3000Hz)
    delay(200);
    noTone(BUZZER_PIN);
  }
}

void vibrationAlarm() {
  for (int i = 0; i < 5; i++) {
    tone(BUZZER_PIN, 4000, 100); // Very high-pitched beep (4000Hz)
    delay(150);
    noTone(BUZZER_PIN);
    delay(50);
  }
}

void updateAndAlertSystem(float tempC) {
  bool tempAbnormal = (tempC < COLD_TEMP) || (tempC > HOT_TEMP);
  bool vibrationAbnormal = (vibrationCount > VIBRATION_THRESHOLD);
  bool gpsAbnormal = geofenceBreached;
  bool currentStatusNormal = !(tempAbnormal || vibrationAbnormal || gpsAbnormal);

  // LED Feedback
  uint32_t color;
  if (tempC < COLD_TEMP) {
    color = strip.Color(0, 0, 255); // Blue
  } else if (tempC > HOT_TEMP) {
    color = strip.Color(255, 0, 0); // Red
  } else {
    color = strip.Color(0, 255, 0); // Green
  }
  strip.fill(color, 0, LED_COUNT);
  strip.show();

  // Consolidated Alert System
  if ((currentStatusNormal != lastStatusNormal) || 
      (!currentStatusNormal && (millis() - lastAlertTime > ALERT_INTERVAL))) {
    
    String report = "📊 *System Report*\n";
    report += TESTING_MODE ? "*TEST MODE*\n" : "";
    report += "🌡️ Temp: " + String(tempC,1) + "°C - ";
    report += tempAbnormal ? "ABNORMAL\n" : "Normal\n";
    
    report += "📳 Vibration: " + String(vibrationCount) + " - ";
    report += vibrationAbnormal ? "FALL DETECTED\n" : "Normal\n";
    
    report += "📍 Location: ";
    #if TESTING_MODE
      report += String(mockLat,6) + "," + String(mockLng,6);
    #else
      report += gps.location.isValid() ? 
        String(gps.location.lat(),6) + "," + String(gps.location.lng(),6) : "Unavailable";
    #endif
    report += " - ";
    report += gpsAbnormal ? "OUTSIDE SAFE ZONE\n" : "Within safe zone\n";
    
    report += "🚦 Overall Status: ";
    report += currentStatusNormal ? "NORMAL" : "ALERT - ABNORMAL DETECTED";
    
    sendAlert(report);
    lastAlertTime = millis();
    lastStatusNormal = currentStatusNormal;
    
    if (!currentStatusNormal) {
      alarmBeep();
      
      if (vibrationAbnormal) {
        vibrationAlarm(); // Special high-pitched alarm for vibrations
      }
      
      if (gpsAbnormal) {
        digitalWrite(MOTOR_PIN, HIGH);
        lastMotorOffTime = millis() + MOTOR_DURATION;
      }
    } else {
      digitalWrite(MOTOR_PIN, LOW);
      noTone(BUZZER_PIN);
    }
  }

  vibrationCount = 0;
}

void logSensorData(float tempC, int vibration) {
  lastLoggedTemp = tempC;
  lastLoggedVibration = vibration;

  double lat, lng;
  #if TESTING_MODE
    lat = mockLat;
    lng = mockLng;
  #else
    lat = gps.location.lat();
    lng = gps.location.lng();
  #endif

  sendToGoogleSheet(tempC, vibration, geofenceBreached, lat, lng);

  Serial.print("Temperature:");
  Serial.print(tempC);
  Serial.print(",Vibration:");
  Serial.println(vibration);

  #if TESTING_MODE
    Serial.print("[DATA] Temp: ");
    Serial.print(tempC);
    Serial.print("°C | Vib: ");
    Serial.println(vibration);
  #endif
}

void checkAndReconnectWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect();
    connectToWiFi();
    secured_client.setInsecure();
  }
}

void sendAlert(String message) {
  bot.sendMessage(CHAT_ID, message, "Markdown");
}

void sendToGoogleSheet(float tempC, int vibration, bool geofenceStatus, double lat, double lng) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected - can't send data");
    return;
  }

  HTTPClient http;
  http.begin(googleScriptUrl);
  http.addHeader("Content-Type", "application/json");

  DynamicJsonDocument doc(256);
  doc["temperature"] = tempC;
  doc["vibration"] = vibration;
  doc["latitude"] = lat;
  doc["longitude"] = lng;
  doc["geofence_breach"] = geofenceStatus;
  doc["device_id"] = "safety_device_01";

  String payload;
  serializeJson(doc, payload);

  int httpResponseCode = http.POST(payload);

  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.println("Google Sheet response: " + response);
  } else {
    Serial.print("Error sending to Google Sheet: ");
    Serial.println(httpResponseCode);
  }

  http.end();
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
      isPlotting = false;
    }
    #if TESTING_MODE
    else if (command == "simulate outside") {
      if (!isPlotting) {
        mockMovingOutside = true;
        Serial.println("Simulating movement outside safe zone...");
      }
    }
    else if (command == "simulate inside") {
      if (!isPlotting) {
        mockMovingOutside = false;
        mockLat = FENCE_LAT;
        mockLng = FENCE_LNG;
        Serial.println("Returned to safe zone");
      }
    }
    else if (command == "set temp") {
      if (!isPlotting) {
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