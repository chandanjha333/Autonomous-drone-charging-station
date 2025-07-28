#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// VL53L0X TOF sensor
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// Pin definitions
#define PUSH_SWITCH 25
#define IR_RECEIVER 33
#define IR_LED 2
#define MOSFET_GATE 18
#define STATUS_LED 21

// Stepper motor pins
#define IN1 12
#define IN2 14
#define IN3 27
#define IN4 26

// Stepper configuration
int step_number = 0;
const int step_sequence[8][4] = {
  {1, 0, 0, 0},
  {1, 1, 0, 0},
  {0, 1, 0, 0},
  {0, 1, 1, 0},
  {0, 0, 1, 0},
  {0, 0, 1, 1},
  {0, 0, 0, 1},
  {1, 0, 0, 1}
};

// Parameters
const int TOF_THRESHOLD = 150; // 15 cm in mm
const int STEP_DELAY = 3;      // ms between steps

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!lox.begin()) {
    Serial.println("Failed to initialize VL53L0X. Check wiring.");
    while (1);
  }

  pinMode(PUSH_SWITCH, INPUT_PULLUP);
  pinMode(IR_RECEIVER, INPUT);
  pinMode(IR_LED, OUTPUT);
  pinMode(MOSFET_GATE, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.println("System ready");
}

void stepMotor(int step) {
  digitalWrite(IN1, step_sequence[step][0]);
  digitalWrite(IN2, step_sequence[step][1]);
  digitalWrite(IN3, step_sequence[step][2]);
  digitalWrite(IN4, step_sequence[step][3]);
}

void movePlatformDown() {
  Serial.println("Lowering platform...");
  while (digitalRead(PUSH_SWITCH) == HIGH) { // until switch pressed
    step_number++;
    if (step_number > 7) step_number = 0;
    stepMotor(step_number);
    delay(STEP_DELAY);
  }
  Serial.println("Platform contact confirmed.");
}

void movePlatformUp(int steps) {
  Serial.println("Raising platform...");
  for (int i = 0; i < steps; i++) {
    step_number--;
    if (step_number < 0) step_number = 7;
    stepMotor(step_number);
    delay(STEP_DELAY);
  }
  Serial.println("Platform raised.");
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);

  if (measure.RangeStatus == 4) {
    Serial.println("Out of range");
    delay(200);
    return;
  }

  Serial.print("Distance (mm): ");
  Serial.println(measure.RangeMilliMeter);

  if (measure.RangeMilliMeter < TOF_THRESHOLD) {
    movePlatformDown(); // lower platform

    delay(2000); // wait 2 seconds

    // Send IR signal
    digitalWrite(IR_LED, HIGH);
    Serial.println("IR signal sent.");

    unsigned long startTime = millis();
    bool charging = false;

    // Wait for IR receiver confirmation
    while (millis() - startTime < 5000) { // 5-second window
      if (digitalRead(IR_RECEIVER) == HIGH) {
        Serial.println("IR confirmation received. Starting charging.");
        digitalWrite(MOSFET_GATE, HIGH); // start charging
        digitalWrite(STATUS_LED, HIGH);
        charging = true;
        break;
      }
    }
    digitalWrite(IR_LED, LOW);

    // Monitor charging
    while (charging) {
      if (digitalRead(IR_RECEIVER) == LOW) { // lost signal
        Serial.println("IR signal lost. Stopping charging.");
        digitalWrite(MOSFET_GATE, LOW);
        digitalWrite(STATUS_LED, LOW);
        charging = false;
        movePlatformUp(4096); // raise platform back (1 revolution)
      }
      delay(200);
    }
  }

  delay(200);
}