#include <Arduino.h>

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
#define L_dist
#define R_dist
#define Front_dist

#define L_sensor_pin 34
#define R_sensor_pin 35
#define Front_sensor_pin 32


#define R_vor_pin 22
#define R_back_pin 23
#define L_vor_pin 19
#define L_back_pin 21

#define Enable_pin 16
#define Disable_pin 4



// ================== Variables ==================
int Drehzahl_sensor_L = 0;
int Drehzahl_sensor_R = 0;

int L_sensor_value = 0;
int R_sensor_value = 0;
int Front_sensor_value = 0;

int L_sensor_old = 0;
int R_sensor_old = 0;

int run = 0;


// ================ Function prototypes ================
void readSensors();
void goStraight();
void turnLeft();
void turnRight();
void followLeftWall();
void followRightWall();
void stopMotors();


void setup() {



    pinMode(Enable_pin, INPUT_PULLUP);
    pinMode(Disable_pin, INPUT_PULLUP);

    pinMode(R_back_pin, OUTPUT);
    pinMode(R_vor_pin, OUTPUT);
    pinMode(L_back_pin, OUTPUT);
    pinMode(L_vor_pin, OUTPUT);

    pinMode(Drehzahl_sensor_L, INPUT);
    pinMode(Drehzahl_sensor_R, INPUT);

    pinMode(L_sensor_pin, INPUT);
    pinMode(R_sensor_pin, INPUT);
    pinMode(Front_sensor_pin, INPUT);

}


void loop() {
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

    Serial.print("L_sensor: "); Serial.print(L_sensor_value);
    Serial.print(" | R_sensor: "); Serial.print(R_sensor_value);
    Serial.print(" | Front_sensor: "); Serial.print(Front_sensor_value);
    
    delay(50); // Small delay for loop stability
}


// ========================= Rsensor Reading ========================
void readSensors() {
    L_sensor_value = analogRead(L_sensor_pin);
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
