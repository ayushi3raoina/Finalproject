#include <TinyGPS++.h>
#include <HardwareSerial.h>

// GPS module on UART1
HardwareSerial SerialGPS(1);
#define GPS_RX 2  // GPS TX --> ESP32 RX1 (GPIO2)
#define GPS_TX 1  // GPS RX --> ESP32 TX1 (GPIO1)

// Vibration motor connected to GPIO 3 (WARNING: shared with USB Serial RX)
#define MOTOR_PIN 3

TinyGPSPlus gps;

// Safe zone center (change as needed)
const double SAFE_LAT = 59.438940;
const double SAFE_LNG = 24.772575;
const double SAFE_RADIUS_METERS = 50.0;

void setup() {
  Serial.begin(115200);                         // Serial monitor
  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);  // GPS module

  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);  // Start with motor OFF

  Serial.println("GPS Safe Zone Monitor Initialized");
  Serial.println("Waiting for GPS signal...");
}

// Print parsed GPS data every 2 seconds
void loop() {
  static unsigned long lastPrint = 0;
  static unsigned long lastAlert = 0;

  while (SerialGPS.available() > 0) {
    char c = SerialGPS.read();
    gps.encode(c);

    // Remove this line to avoid raw NMEA flooding
    // Serial.write(c);
  }

  if (gps.location.isValid() && gps.location.isUpdated()) {
    // Print every 2 seconds
    if (millis() - lastPrint > 2000) {
      lastPrint = millis();

      double lat = gps.location.lat();
      double lng = gps.location.lng();
      double distance = TinyGPSPlus::distanceBetween(lat, lng, SAFE_LAT, SAFE_LNG);

      Serial.print("\nSatellites: ");
      Serial.println(gps.satellites.value());

      Serial.print("Fix Type: ");
      if (gps.location.age() == 0) Serial.println("FRESH FIX");
      else if (gps.location.age() == 255) Serial.println("NO FIX");
      else Serial.println("STALE FIX");

      Serial.print("Lat: ");
      Serial.println(lat, 6);
      Serial.print("Lng: ");
      Serial.println(lng, 6);

      Serial.print("Distance from safe zone: ");
      Serial.print(distance, 2);
      Serial.println(" meters");

      // Trigger vibration motor if outside safe zone
      if (distance > SAFE_RADIUS_METERS) {
        Serial.println("OUTSIDE safe zone - MOTOR ON");
        digitalWrite(MOTOR_PIN, HIGH);
      } else {
        Serial.println("INSIDE safe zone - MOTOR OFF");
        digitalWrite(MOTOR_PIN, LOW);
      }
    }
  }

  // Alert if no satellites every 5 seconds
  if (millis() - lastAlert > 5000) {
    lastAlert = millis();
    if (gps.satellites.value() == 0) {
      Serial.println("\n⚠️ NO SATELLITES DETECTED - check GPS module & antenna");
    }
  }
}
