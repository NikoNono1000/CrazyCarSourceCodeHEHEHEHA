#include <Arduino.h>

// --- Pin definitions (change as needed) ---
// PWM outputs for motor speed (use LEDC channels)
const int MOTOR_A_PWM_PIN = 18; // PWM pin for Motor A (ledc channel 0)
const int MOTOR_B_PWM_PIN = 19; // PWM pin for Motor B (ledc channel 1)

// Direction pins for motors (H-bridge direction inputs)
const int MOTOR_A_DIR_PIN = 5;
const int MOTOR_B_DIR_PIN = 17;

// IR sensors (analog-capable pins recommended)
const int IR_FRONT_PIN = 34; // frontal sensor (ADC1)
const int IR_LEFT_PIN  = 35; // left-45deg sensor (ADC1)
const int IR_RIGHT_PIN = 32; // right-45deg sensor (ADC1)

// PWM / LEDC settings
const int MOTOR_A_CHANNEL = 0;
const int MOTOR_B_CHANNEL = 1;
const int PWM_FREQ = 20000; // 20 kHz
const int PWM_RESOLUTION = 8; // 8-bit resolution (0-255)
const int MAX_DUTY = (1 << PWM_RESOLUTION) - 1; // 255

// Behavior tuning
const int BASE_SPEED_PERCENT = 60; // default forward speed (0-100)
int FRONT_THRESHOLD = 1500; // ADC threshold for frontal obstacle (0-4095)
int SIDE_THRESHOLD  = 1500; // ADC threshold for left/right sensors

// Helper: set motor speed and direction
void setMotor(int channel, int dirPin, int speedPercent, bool forward) {
  speedPercent = constrain(speedPercent, 0, 100);
  int duty = map(speedPercent, 0, 100, 0, MAX_DUTY);
  digitalWrite(dirPin, forward ? HIGH : LOW);
  ledcWrite(channel, duty);
}

void stopMotors() {
  ledcWrite(MOTOR_A_CHANNEL, 0);
  ledcWrite(MOTOR_B_CHANNEL, 0);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("ESP32 Motor+IR starting...");

  // Configure direction pins
  pinMode(MOTOR_A_DIR_PIN, OUTPUT);
  pinMode(MOTOR_B_DIR_PIN, OUTPUT);

  // Configure LEDC PWM channels and attach pins
  ledcSetup(MOTOR_A_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_A_PWM_PIN, MOTOR_A_CHANNEL);

  ledcSetup(MOTOR_B_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_B_PWM_PIN, MOTOR_B_CHANNEL);

  // Print initial sensor values to help calibration
  Serial.println("Initial sensor readings (calibrate thresholds if needed):");
  delay(100);
  int f = analogRead(IR_FRONT_PIN);
  int l = analogRead(IR_LEFT_PIN);
  int r = analogRead(IR_RIGHT_PIN);
  Serial.printf(" Front=%d  Left=%d  Right=%d\n", f, l, r);
}

void loop() {
  // --- Start/Stopp Bedingungen prüfen ---
  // START_PIN: Taster (aktiv LOW) zum Starten/Stoppen
  // EMERGENCY_PIN: Notausschalter (aktiv LOW)
  const int START_PIN = 14;      // anpassen falls erforderlich
  const int EMERGENCY_PIN = 13;  // anpassen falls erforderlich

  // Not-Aus prüfen (höchste Priorität)
  if (digitalRead(EMERGENCY_PIN) == LOW) {
    stopMotors();
    Serial.println("NOT-AUS aktiviert: Motoren gestoppt");
    delay(100);
    return; // nichts weiter tun solange Notausschalter aktiv ist
  }

  bool started = (digitalRead(START_PIN) == LOW); // Taster aktiv LOW
  if (!started) {
    // Wenn nicht gestartet, Motoren anhalten und nichts weiter tun
    stopMotors();
    return;
  }

  // --- IR Sensoren auslesen ---
  int frontRaw = analogRead(IR_FRONT_PIN); // 0..4095
  int leftRaw  = analogRead(IR_LEFT_PIN);
  int rightRaw = analogRead(IR_RIGHT_PIN);

  // --- Daten aus Sensoredaten berechnen ---
  // Wir berechnen hier eine einfache Näherungs-"Proximity"-Skala 0..100
  int frontPct = map(frontRaw, 0, 4095, 0, 100);
  int leftPct  = map(leftRaw,  0, 4095, 0, 100);
  int rightPct = map(rightRaw, 0, 4095, 0, 100);

  // Debug: kurz ausgeben
  static unsigned long lastDbg = 0;
  if (millis() - lastDbg > 500) {
    Serial.printf("Sensoren (raw): F=%d L=%d R=%d | prox: F=%d L=%d R=%d\n",
                  frontRaw, leftRaw, rightRaw, frontPct, leftPct, rightPct);
    lastDbg = millis();
  }

  // --- Motoren per PWM ansteuern ---
  // Einfache reaktive Logik basierend auf Schwellenwerten
  bool obstacleFront = (frontRaw >= FRONT_THRESHOLD);
  bool obstacleLeft  = (leftRaw  >= SIDE_THRESHOLD);
  bool obstacleRight = (rightRaw >= SIDE_THRESHOLD);

  if (!obstacleFront) {
    // Weg frei -> vorwärts
    setMotor(MOTOR_A_CHANNEL, MOTOR_A_DIR_PIN, BASE_SPEED_PERCENT, true);
    setMotor(MOTOR_B_CHANNEL, MOTOR_B_DIR_PIN, BASE_SPEED_PERCENT, true);
  } else {
    // Hindernis vorn
    if (obstacleLeft && !obstacleRight) {
      // Hindernis vorn+links -> nach rechts drehen
      Serial.println("Ausweichmanöver: Rechts drehen (Hindernis vorn+links)");
      setMotor(MOTOR_A_CHANNEL, MOTOR_A_DIR_PIN, BASE_SPEED_PERCENT, true);
      setMotor(MOTOR_B_CHANNEL, MOTOR_B_DIR_PIN, BASE_SPEED_PERCENT, false);
    } else if (obstacleRight && !obstacleLeft) {
      // Hindernis vorn+rechts -> nach links drehen
      Serial.println("Ausweichmanöver: Links drehen (Hindernis vorn+rechts)");
      setMotor(MOTOR_A_CHANNEL, MOTOR_A_DIR_PIN, BASE_SPEED_PERCENT, false);
      setMotor(MOTOR_B_CHANNEL, MOTOR_B_DIR_PIN, BASE_SPEED_PERCENT, true);
    } else {
      // Hindernis überall oder nur vorn -> kurz rückwärts und drehen
      Serial.println("Ausweichmanöver: Rückwärts + Drehen (Hindernis vorn)");
      setMotor(MOTOR_A_CHANNEL, MOTOR_A_DIR_PIN, 50, false);
      setMotor(MOTOR_B_CHANNEL, MOTOR_B_DIR_PIN, 50, false);
      delay(300);
      // Drehung auf der Stelle
      setMotor(MOTOR_A_CHANNEL, MOTOR_A_DIR_PIN, BASE_SPEED_PERCENT, true);
      setMotor(MOTOR_B_CHANNEL, MOTOR_B_DIR_PIN, BASE_SPEED_PERCENT, false);
      delay(300);
    }
  }

  // Kurze Verzögerung für Regelschleife
  delay(20);
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
} 