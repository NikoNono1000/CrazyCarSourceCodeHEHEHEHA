#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h> // Adafruit-Version des VL53L0X-Treibers verwenden

//=================== Velocity ==================
#define speed 255

#define M_scale_L 0.648 // if scale R is 1
#define M_scale_R 1.543 // if scale L is 1

#define L_speed speed * M_scale_L          
#define R_speed speed

#define turn_speed 180
#define turn_time 500

#define L_turn_speed turn_speed*M_scale_L
#define R_turn_speed turn_speed



// ================== Distances ==================
#define Des_dist 15 
    // desired distance in cm
// 1 cm in L_sid_dis value = 
// 1 cm in R_sid_dis value = 
#define Front 800
    // desirent front dis in m
// 1 cm in front value = 

#define n_default 2
#define n_t 5



// ================== Pins ==================
//#define L_dist   // entfällt
#define R_dist
#define Front_dist

//#define L_sensor_pin 32   // entfällt
#define R_sensor_pin 34
#define Front_sensor_pin 35

#define SDA_PIN 14
#define SCL_PIN 27


#define R_vor_pin 22
#define R_back_pin 23
#define L_vor_pin 19
#define L_back_pin 21

#define Enable_pin 16
#define Disable_pin 4



// ================== Variables ==================
volatile int run = 0; // <--- Hinzugefügt

// int Drehzahl_sensor_L = 0;
// int Drehzahl_sensor_R = 0;

//int L_sensor_value = 0;   // entfällt
int R_sensor_value = 0;
int Front_sensor_value = 0;

//int L_sensor_old = 0;     // entfällt
int R_sensor_old = 0;

int L_sensor_value = 0; // digitaler Wert (in cm)
int L_sensor_old = 0;

// ================ I2C Sensor Objekt ================
Adafruit_VL53L0X l_sensor; // Adafruit-Objekt verwenden

// ================ Function prototypes ================
void readSensors();
void goStraight();
void turnLeft();
void turnRight();
void followLeftWall();
void followRightWall();
void stopMotors();


void setup() {
    Serial.begin(115200);

    // I2C für digitalen Sensor initialisieren
    Wire.begin(SDA_PIN, SCL_PIN);
    if (!l_sensor.begin()) {
        Serial.println("Fehler: VL53L0X nicht gefunden!");
        while (1);
    }
    // l_sensor.setTimeout(500); // Nicht in Adafruit-Lib
    // if (!l_sensor.init()) { ... }
    // l_sensor.startContinuous(); // Nicht in Adafruit-Lib

    pinMode(R_back_pin, OUTPUT);
    pinMode(R_vor_pin, OUTPUT);
    pinMode(L_back_pin, OUTPUT);
    pinMode(L_vor_pin, OUTPUT);

    pinMode(Enable_pin, INPUT_PULLUP);   // <--- Hinzugefügt
    pinMode(Disable_pin, INPUT_PULLUP);  // <--- Hinzugefügt

    // Entferne pinMode für die analogen Sensorpins!
}


void loop() {
    // Test: Nur jeweils einen Sensor anschließen und die Werte beobachten!
    // 1. Nur rechter Sensor angeschlossen: Wie verhalten sich L_raw und R_raw?
    // 2. Nur linker Sensor angeschlossen: Wie verhalten sich L_raw und R_raw?
    // 3. Nur Frontsensor angeschlossen: Wie verhalten sich F_raw?

    // Debug-Ausgabe: Sensorwerte und Warnung bei dauerhaft 0
    Serial.print("L_digital: "); Serial.print(L_sensor_value);
    if (L_sensor_value == 0) Serial.print(" [WARN: L=0]");
    Serial.print(" | R_raw: "); Serial.print(analogRead(R_sensor_pin));
    if (analogRead(R_sensor_pin) == 0) Serial.print(" [WARN: R=0]");
    Serial.print(" | F_raw: "); Serial.println(analogRead(Front_sensor_pin));
    delay(50); // <--- vorher 500, jetzt 50 für schnellere Messungen

    if (digitalRead(Enable_pin) == LOW) { // Button gedrückt (LOW)
        run = 1;
    }
    if (digitalRead(Disable_pin) == LOW) { // Button gedrückt (LOW)
        run = 0;
        stopMotors();
    }


    if (run == 1) {
            // Read all sensor values
            readSensors();
            
            // Decision logic
            // 1. If front sensor detects obstacle is far enough, check side sensors
        if (Front_sensor_value < Front) {
            
            // 2. Check for new openings (turns)
            if (L_sensor_old * n_t < L_sensor_value) {
                // New opening on left detected - turn left
                turnLeft();
                delay(turn_time);
                followLeftWall();
            }
            else if (R_sensor_old * n_t < R_sensor_value) {
                // New opening on right detected - turn right
                turnRight();
                delay(turn_time);
                followRightWall();
            }
            // 3. Follow the closer wall
            else if (L_sensor_value * n_default < R_sensor_value) {
                // Left wall is closer - follow left
                followLeftWall();
            }
            else if (R_sensor_value * n_default < L_sensor_value) {
                // Right wall is closer - follow right
                followRightWall();
            }
            else {
                // Both walls equidistant or no walls - go straight
                goStraight();
            }
        }
        else {
            // Front obstacle too close - stop or turn
            stopMotors();

            
            // Decide which way to turn based on side distances
            if (L_sensor_value > R_sensor_value) {
                turnLeft();
                delay(turn_time);
            }
            else {
                turnRight();
                delay(turn_time);
            }
        }
    }
    
    // Store current sensor values as old values for next iteration
    L_sensor_old = L_sensor_value;
    R_sensor_old = R_sensor_value;

    delay(10); // <--- vorher 50, jetzt 10 für noch schnellere Schleifen
}


// ========================= Rsensor Reading ========================
void readSensors() {
    // Digitalen linken Sensor auslesen (in mm, umrechnen in cm)
    uint16_t range = l_sensor.readRange();
    if (!l_sensor.timeoutOccurred()) {
        L_sensor_value = range / 10;
    } else {
        L_sensor_value = 0;
    }
    R_sensor_value = analogRead(R_sensor_pin);
    Front_sensor_value = analogRead(Front_sensor_pin);
}


// ========================= straight forward ========================
void goStraight() {
    analogWrite(L_vor_pin, L_speed);
    analogWrite(R_vor_pin, R_speed);
    digitalWrite(L_back_pin, LOW);
    digitalWrite(R_back_pin, LOW);
}


// Turn left (left velocity forwared slower, right velocity forward default)
void turnLeft() {
    analogWrite(L_vor_pin, L_turn_speed);
    analogWrite(R_vor_pin, R_speed);
    digitalWrite(L_back_pin, LOW);
    digitalWrite(R_back_pin, LOW);

}


// Turn right (right velocity forwared slower, left velocity forward default)
void turnRight() {
    analogWrite(R_vor_pin, R_turn_speed);
    analogWrite(L_vor_pin, L_speed);
    digitalWrite(R_back_pin, LOW);
    digitalWrite(L_back_pin, LOW);
}


// Follow left wall - adjust to maintain desired distance
void followLeftWall() {
    if (L_sensor_value < Des_dist) {
        // Too close to left wall - steer slightly right
        analogWrite(L_vor_pin, L_speed);
        analogWrite(R_vor_pin, R_speed * 0.8);
    }
    else if (L_sensor_value > Des_dist * 1.5) {
        // Too far from left wall - steer slightly left
        analogWrite(L_vor_pin, L_speed * 0.8);
        analogWrite(R_vor_pin, R_speed);
    }
    else {
        // Good distance - go straightv slo
        goStraight();
    }
    digitalWrite(L_back_pin, LOW);
    digitalWrite(R_back_pin, LOW);
}


// Follow right wall - adjust to maintain desired distance
void followRightWall() {
    if (R_sensor_value < Des_dist) {
        // Too close to right wall - steer slightly left
        analogWrite(L_vor_pin, L_speed * 0.8);
        analogWrite(R_vor_pin, R_speed );
    }
    else if (R_sensor_value > Des_dist * 1.5) {
        // Too far from right wall - steer slightly right
        analogWrite(L_vor_pin, L_speed);
        analogWrite(R_vor_pin, R_speed * 0.8);
    }
    else {
        // Good distance - go straight
        goStraight();
    }
    digitalWrite(L_back_pin, LOW);
    digitalWrite(R_back_pin, LOW);
}


// Stop all motors
void stopMotors() {
    analogWrite(L_vor_pin, 0);
    analogWrite(R_vor_pin, 0);
    digitalWrite(L_back_pin, LOW);
    digitalWrite(R_back_pin, LOW);
}
