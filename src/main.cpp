#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// ================== PINBELEGUNG ==================
#define SDA_PIN 14
#define SCL_PIN 27

#define R_SENSOR_PIN 34
#define F_SENSOR_PIN 35

#define L_VOR 19
#define L_BACK 21
#define R_VOR 22
#define R_BACK 23

#define BTN_START 16
#define BTN_STOP  4

// ================== PARAMETER ==================
#define BASE_SPEED 180
#define TURN_SPEED 140

#define DES_DIST 15
#define FRONT_LIMIT 18

// ==== SENSORSCHWELLEN (Empfindlichkeit) ====
const int MIN_LEFT_DIST = 17;      // zu nah an linker Wand
const int MAX_LEFT_DIST = 23;      // zu weit weg von linker Wand
const int MIN_FRONT_DIST = 50;     // zu nah an Hindernis vorne

// ================== PWM ==================
#define PWM_FREQ 20000
#define PWM_RES 8

#define CH_L 0
#define CH_R 1

// ================== GLOBALE VARIABLEN ==================
bool run = false;

int L_dist = 0;
int R_dist = 0;
int F_dist = 0;

Adafruit_VL53L0X vl53;

// ================== FUNKTIONS-PROTOTYPEN ==================
void readSensors();
void debugPrint();

void drive(int l, int r);
void stopMotors();
void turnRight();
void followLeftWall();

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  Serial.println("Serial Monitor bereit. Sensorwerte werden ausgegeben.");

  Wire.begin(SDA_PIN, SCL_PIN);
  if (!vl53.begin()) {
    Serial.println("VL53L0X NICHT GEFUNDEN");
    while (1);
  }

  pinMode(L_BACK, OUTPUT);
  pinMode(R_BACK, OUTPUT);

  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_STOP, INPUT_PULLUP);

  ledcSetup(CH_L, PWM_FREQ, PWM_RES);
  ledcSetup(CH_R, PWM_FREQ, PWM_RES);

  ledcAttachPin(L_VOR, CH_L);
  ledcAttachPin(R_VOR, CH_R);

  stopMotors();
}

// ================== LOOP ==================
void loop() {
  readSensors();
  debugPrint();

  if (digitalRead(BTN_START) == LOW) run = true;
  if (digitalRead(BTN_STOP) == LOW) {
    run = false;
    stopMotors();
  }

  if (!run) return;

  if (F_dist < FRONT_LIMIT) {
    stopMotors();
    turnRight();
    delay(400);
    return;
  }

  followLeftWall();
  delay(20);
}

// ================== SENSORLESEN ==================
void readSensors() {
  uint16_t mm = vl53.readRange();
  if (!vl53.timeoutOccurred()) {
    L_dist = mm / 10;
  }

  R_dist = map(analogRead(R_SENSOR_PIN), 0, 4095, 80, 5);
  F_dist = map(analogRead(F_SENSOR_PIN), 0, 4095, 80, 5);
}

// ================== DEBUG ==================
void debugPrint() {
  Serial.print("R: "); Serial.print(R_dist);
  Serial.print(" | L: "); Serial.print(L_dist);
  Serial.print(" | F: "); Serial.println(F_dist);
}

// ================== MOTORLOGIK ==================
void drive(int l, int r) {
  digitalWrite(L_BACK, LOW);
  digitalWrite(R_BACK, LOW);
  ledcWrite(CH_L, l);
  ledcWrite(CH_R, r);
}

void stopMotors() {
  ledcWrite(CH_L, 0);
  ledcWrite(CH_R, 0);
}

void turnRight() {
  drive(TURN_SPEED, BASE_SPEED);
}

// ================== WAND FOLGEN ==================
void followLeftWall() {
  int left = L_dist;
  int front = F_dist;

  // Sehr aggressives Einlenken nach rechts, wenn zu nah an linker Wand oder vorne zu nah
  if (left < MIN_LEFT_DIST || front < MIN_FRONT_DIST) {
    drive(BASE_SPEED * 0.15, BASE_SPEED);
    return;
  }

  // Sehr aggressives Einlenken nach links, wenn zu weit weg von linker Wand
  if (left > MAX_LEFT_DIST) {
    drive(BASE_SPEED, BASE_SPEED * 0.15);
    return;
  }

  // Im optimalen Bereich -> geradeaus
  drive(BASE_SPEED, BASE_SPEED);
}
