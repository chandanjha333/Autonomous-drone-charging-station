#include <Arduino.h>

// Pin definitions
#define BATTERY_ADC A0     // PB5 - Battery voltage divider
#define IR_RECEIVER PB3    // Digital input
#define IR_LED PB4         // IR LED output
#define STATUS_LED PB0     // Charging indicator
#define MOSFET_ENABLE PB1  // Optional charging control

// Voltage thresholds (adjust for your divider)
const float V_REF = 1.1;          // Internal reference voltage
const float R1 = 100000.0;        // Voltage divider top resistor
const float R2 = 33000.0;         // Voltage divider bottom resistor
const float BATTERY_CUTOFF = 4.1; // Stop charging above this voltage

void setup() {
  pinMode(IR_RECEIVER, INPUT);
  pinMode(IR_LED, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);
  pinMode(MOSFET_ENABLE, OUTPUT);

  analogReference(INTERNAL); // Use 1.1V internal reference
}

float readBatteryVoltage() {
  uint16_t adcValue = analogRead(BATTERY_ADC);
  float voltage = (adcValue / 1023.0) * V_REF;
  return voltage * ((R1 + R2) / R2); // Convert using voltage divider ratio
}

void loop() {
  float batteryVoltage = readBatteryVoltage();

  if (batteryVoltage < BATTERY_CUTOFF) {
    // Battery needs charging
    if (digitalRead(IR_RECEIVER) == HIGH) {
      // TX is sending IR pulse; reply
      digitalWrite(IR_LED, HIGH);
      delay(5); // short pulse
      digitalWrite(IR_LED, LOW);
    }

    digitalWrite(STATUS_LED, HIGH);      // Charging indicator ON
    digitalWrite(MOSFET_ENABLE, HIGH);   // Enable TP4056 or MOSFET
  } else {
    // Battery full
    digitalWrite(IR_LED, LOW);           // No response to TX
    digitalWrite(STATUS_LED, LOW);       // Charging indicator OFF
    digitalWrite(MOSFET_ENABLE, LOW);    // Disable charging
  }

  delay(100); // Small delay to avoid busy-waiting
}