// =======================
// ESP32 Kettenfahrzeug
// Geradeaus fahren
// =======================
#include <Arduino.h>

// Motor links (54 mm)
#define PWM_LEFT   25
#define DIR_LEFT   26

// Motor rechts (35,01 mm)
#define PWM_RIGHT  27
#define DIR_RIGHT  14

// Buttons
#define BTN_START  32
#define BTN_STOP   33

// Parameter
#define BASE_SPEED 120
#define SCALE_SMALL 1.543

bool driving = false;

// PWM Kanäle
#define CH_LEFT  0
#define CH_RIGHT 1

void setup() {
  // Motorpins
  pinMode(DIR_LEFT, OUTPUT);
  pinMode(DIR_RIGHT, OUTPUT);

  // Buttons
  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_STOP, INPUT_PULLUP);

  // PWM Setup ESP32
  ledcSetup(CH_LEFT,  20000, 8); // 20 kHz, 8 Bit
  ledcSetup(CH_RIGHT, 20000, 8);

  ledcAttachPin(PWM_LEFT,  CH_LEFT);
  ledcAttachPin(PWM_RIGHT, CH_RIGHT);

  stopMotors();
}

void loop() {
  // Start gedrückt?
  if (digitalRead(BTN_START) == LOW) {
    driving = true;
  }

  // Stop gedrückt?
  if (digitalRead(BTN_STOP) == LOW) {
    driving = false;
  }

  if (driving) {
    driveForward(BASE_SPEED);
  } else {
    stopMotors();
  }

  delay(10); // Entprellung light
}

// =======================
// Funktionen
// =======================

void driveForward(int speed) {
  int speedLeft  = speed;
  int speedRight = speed * SCALE_SMALL;

  speedRight = constrain(speedRight, 0, 255);

  digitalWrite(DIR_LEFT, HIGH);   // Richtung vorwärts
  digitalWrite(DIR_RIGHT, HIGH);

  ledcWrite(CH_LEFT,  speedLeft);
  ledcWrite(CH_RIGHT, speedRight);
}

void stopMotors() {
  ledcWrite(CH_LEFT,  0);
  ledcWrite(CH_RIGHT, 0);
}
