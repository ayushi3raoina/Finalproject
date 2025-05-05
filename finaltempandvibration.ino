#include <Adafruit_NeoPixel.h>

// ===== Pin Definitions =====
#define VIBRATION_PIN 3      // SW-520D "DO" connected to pin 3
#define BUZZER_PIN 26        // Buzzer connected to pin 26
#define LED_PIN 4            // NeoPixel data pin
#define TMP36_PIN 2          // TMP36 analog pin (GPIO34)

// ===== Constants =====
#define LED_COUNT 12         // Number of NeoPixels
#define BRIGHTNESS 50        // NeoPixel brightness (0-255)
#define VIBRATION_THRESHOLD 100  // Vibration count threshold
#define COLD_TEMP 20.0       // Blue below this (°C)
#define HOT_TEMP 34.0        // Red above this (°C)

// ===== Global Variables =====
int vibrationCount = 0;
unsigned long lastCheckTime = 0;
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  // Initialize pins
  pinMode(VIBRATION_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  
  // Initialize NeoPixel
  strip.begin();
  strip.setBrightness(BRIGHTNESS);
  strip.show(); // Turn off all pixels
  
  // Configure ADC for TMP36
  analogReadResolution(12); // 12-bit precision (0-4095)
  
  // Initialize serial for debugging
  Serial.begin(115200);
  Serial.println("System started - Monitoring vibration and temperature");
}

void loop() {
  // ===== Vibration Monitoring =====
  int sensorValue = digitalRead(VIBRATION_PIN);
  
  if (sensorValue == LOW) {
    vibrationCount++;
    delay(10); // debounce delay
  }

  // ===== Temperature Monitoring =====
  // Read raw ADC value
  int rawValue = analogRead(TMP36_PIN);
  
  // Convert to voltage (0-3.3V range)
  float voltage = rawValue * (3.3 / 4095.0);
  
  // Calculate temperature (TMP36 formula: 10mV/°C + 500mV offset)
  float tempC = (voltage - 0.5) * 100.0;

  // ===== System Update (every 1 second) =====
  if (millis() - lastCheckTime > 1000) {
    // Print debug information
    Serial.print("Vibration count: ");
    Serial.print(vibrationCount);
    Serial.print(" | Temp: ");
    Serial.print(tempC);
    Serial.println("°C");

    // Check vibration threshold
    if (vibrationCount > VIBRATION_THRESHOLD) {
      alarmBeep();
      Serial.println("ALARM - Vibration threshold exceeded!");
    } else {
      noTone(BUZZER_PIN);
    }

    // Update NeoPixels based on temperature
    uint32_t color;
    if (tempC < COLD_TEMP) {
      color = strip.Color(0, 0, 255); // Blue (cold)
    } 
    else if (tempC < HOT_TEMP) {
      color = strip.Color(0, 255, 0); // Green (normal)
    } 
    else {
      color = strip.Color(255, 0, 0); // Red (hot)
      // If temperature is too hot, also trigger alarm
      alarmBeep();
      Serial.println("ALARM - High temperature detected!");
    }

    // Update all LEDs
    for (int i = 0; i < LED_COUNT; i++) {
      strip.setPixelColor(i, color);
    }
    strip.show();
    
    // Reset counters for next interval
    vibrationCount = 0;
    lastCheckTime = millis();
  }
}

// Buzzer alarm pattern
void alarmBeep() {
  // Play multiple beeps
  for (int i = 0; i < 3; i++) {
    tone(BUZZER_PIN, 2000);  // 2000Hz tone
    delay(200);              // ON for 200ms
    noTone(BUZZER_PIN);      // Turn off
    delay(200);              // Pause for 200ms
  }
}
