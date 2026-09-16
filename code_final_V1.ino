#include <DynamixelSerial2.h>
#include <Arduino.h>
#include <math.h>

// Constants and Definitions
#define BAUDS 9600
#define STANDARD_SPEED 500
#define ABSOLUTE_MODE 0

#define STEP_PIN_1 2
#define DIR_PIN_1 3
#define ENABLE_PIN_1 4

#define STEP_PIN_4 5
#define DIR_PIN_4 6
#define ENABLE_PIN_4 7

#define STEP_PIN_0 8
#define DIR_PIN_0 9
#define ENABLE_PIN_0 10

#define PIN_CONTROL 11

#define ENDSTOP_PIN_1 12
#define ENDSTOP_PIN_2 13
#define ENDSTOP_PIN_3 14

const double PI = 3.141592653589793;
const double l_1 = 0.24, l_2 = 0.21, l_3 = 0.14, l_4 = 50;

int mode_flag = ABSOLUTE_MODE;
int stepsTour = 200, micropas = 8, K = 193;

// Dynamixel Variables
int id = 12;  
double x = 0, y = 0, z = 0, px = 0, py = 0, pz = 0;
double dx, dy, dz, theta_1, theta_2, theta_3, theta_0, theta_4;

void setup() {
    // Pin Modes
    pinMode(STEP_PIN_1, OUTPUT);
    pinMode(DIR_PIN_1, OUTPUT);
    pinMode(ENABLE_PIN_1, OUTPUT);
    pinMode(STEP_PIN_4, OUTPUT);
    pinMode(DIR_PIN_4, OUTPUT);
    pinMode(ENABLE_PIN_4, OUTPUT);
    pinMode(STEP_PIN_0, OUTPUT);
    pinMode(DIR_PIN_0, OUTPUT);
    pinMode(ENABLE_PIN_0, OUTPUT);

    pinMode(ENDSTOP_PIN_1, INPUT);
    pinMode(ENDSTOP_PIN_2, INPUT);
    pinMode(ENDSTOP_PIN_3, INPUT);

    // Enable Motors
    digitalWrite(ENABLE_PIN_1, LOW);
    digitalWrite(ENABLE_PIN_4, LOW);
    digitalWrite(ENABLE_PIN_0, LOW);

    // Initialize Serial and Dynamixel
    Serial.begin(BAUDS);
    Dynamixel.begin(1000000, 2); // Adjust baudrate and pin as needed
    Serial.println("System Initialized. Ready for commands.");
}

void loop() {
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        handleCommand(cmd);
    }
}

void handleCommand(char cmd) {
    switch (cmd) {
        case 'M': // Movement Mode
            mode_flag = parseNumber('M', ABSOLUTE_MODE);
            handleMovement();
            break;
        case '?': // Help Command
            printHelp();
            break;
        case 'i': // Reinitialize Motors
            initialiseMotors();
            break;
        default:
            Serial.println("Invalid Command");
    }
}

void handleMovement() {
  if (mode_flag==0){
    x = parseNumber('X', px);
    y = parseNumber('Y', py);
    z = parseNumber('Z', pz);

    dx = x - px;
    dy = y - py;
    dz = z - pz;

    theta_0 = calculateTheta0(dy, dx);
    theta_1 = calculateTheta1(dx, dy, dz);
    theta_2 = calculateTheta2(dx, dy, dz);
    theta_4 = calculateTheta4(theta_1, theta_2);
    theta_3 = calculateTheta3(theta_1, theta_2);

    moveMotors(theta_1, theta_4, theta_3, theta_0);

    px = x;
    py = y;
    pz = z;
}
  if (mode_flag==1)  // relative mode 
    {
    dx = parseNumber('X', 0);
    dy = parseNumber('Y', 0);
    dz = parseNumber('Z', 0);
      theta_0 = calculateTheta0(dy, dx);
      theta_1 = calculateTheta1(dx, dy, dz);
      theta_2 = calculateTheta2(dx, dy, dz);
      theta_4 = calculateTheta4(theta_1, theta_2);
      theta_3 = calculateTheta3(theta_1, theta_2);
          pz+=dz;
          py+=dy;
          px+=dx;
    }
}
void moveMotors(double T1, double T4, double T3, double T0) {
    int steps_0 = (abs(T0) / 360.0) * stepsTour * micropas;
    int steps_1 = (abs(T1) / 360.0) * stepsTour * micropas;
    int steps_4 = (abs(T4) / 360.0) * stepsTour * micropas;
    int angle3 = (T3 / 360.0) * 1023;

    digitalWrite(DIR_PIN_0, T0 > 0 ? HIGH : LOW);
    digitalWrite(DIR_PIN_1, T1 > 0 ? HIGH : LOW);
    digitalWrite(DIR_PIN_4, T4 > 0 ? HIGH : LOW);

    for (int i = 1; i <= steps_1; i++) {
        digitalWrite(STEP_PIN_1, HIGH);
        delayMicroseconds(500);
        digitalWrite(STEP_PIN_1, LOW);
        delayMicroseconds(500);
    }

    for (int i = 1; i <= steps_4; i++) {
        digitalWrite(STEP_PIN_4, HIGH);
        delayMicroseconds(500);
        digitalWrite(STEP_PIN_4, LOW);
        delayMicroseconds(500);
    }

    for (int i = 1; i <= steps_0; i++) {
        digitalWrite(STEP_PIN_0, HIGH);
        delayMicroseconds(500);
        digitalWrite(STEP_PIN_0, LOW);
        delayMicroseconds(500);
    }

    Dynamixel.moveSpeed(id, angle3, 60 * (1023 / 360));
}

double parseNumber(char key, double defaultValue) {
    // Placeholder for actual parsing logic
    return defaultValue;
}

void printHelp() {
    Serial.println("Available Commands:");
    Serial.println("M: Movement Mode (Absolute or Relative)");
    Serial.println("i: Initialize Motors");
    Serial.println("?: Show Help");
}

void initialiseMotors() {
    digitalWrite(DIR_PIN_0, LOW);
    digitalWrite(DIR_PIN_4, LOW);
    digitalWrite(DIR_PIN_1, LOW);

    while (digitalRead(ENDSTOP_PIN_1) == LOW) {
        digitalWrite(STEP_PIN_0, HIGH);
        delayMicroseconds(200);
        digitalWrite(STEP_PIN_0, LOW);
        delayMicroseconds(200);
    }
    Serial.println("Base motor initialized to zero.");

    while (digitalRead(ENDSTOP_PIN_2) == LOW) {
        digitalWrite(STEP_PIN_1, HIGH);
        delayMicroseconds(200);
        digitalWrite(STEP_PIN_1, LOW);
        delayMicroseconds(200); 
    }
    Serial.println("Arm motor initialized to zero.");

    while (digitalRead(ENDSTOP_PIN_3) == LOW) {
        digitalWrite(STEP_PIN_4, HIGH);
        delayMicroseconds(200);
        digitalWrite(STEP_PIN_4, LOW);
        delayMicroseconds(200); 
    }
    Serial.println("Wrist motor initialized to zero.");
}

// Kinematic Calculation Functions
double calculateTheta0(double dy, double dx) {
    return atan2(dy, dx) * (180.0 / PI); // Converts radians to degrees
}

double calculateTheta1(double dx, double dy, double dz) {
    double r = sqrt(dx * dx + dy * dy+dz * dz);
    double alpha = atan2(dz, sqrt(dx*dx + dy*dy));
    double term1 = (r * r + l_1 * l_1 - l_2 * l_2) / (2 * l_1 * r);
    double cosTerm1 = constrain(term1, -1.0, 1.0);
    return  (360 / (2 * PI)) * (acos(cosTerm1) - alpha);
}

double calculateTheta2(double dx, double dy, double dz) {
    double r = sqrt(dx * dx + dy * dy + dz * dz);
    return acos(((l_1 * l_1 + l_2 * l_2 - r * r-dz * dz)) / (2 * l_1 * l_2)) * (180.0 / PI);
}

double calculateTheta3(double theta1, double theta2) {
    return 180 - theta1 - theta2; // Simplified for example
}

double calculateTheta4(double theta1, double theta2) {
    return theta1 + theta2-180; // Simplified for example
}
