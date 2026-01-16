#pragma once

#include <Arduino.h>

// Motor pin definitions - L298N H-bridge
const int MOTOR_PIN[2][2] = {{9, 10}, {25, 26}}; // [side][channel] side: 0=left, 1=right

// Motor control modes
enum MotorMode {
    MODE_PIN_TEST = 0,
    MODE_SIMPLE_DRIVE = 1,
    MODE_JOYSTICK = 2
};

// Global variables
MotorMode currentMode = MODE_JOYSTICK; // Default to joystick mode
bool motorPinStates[4] = {false, false, false, false}; // Logical ON/OFF (pin test mode)
uint16_t motorDuty[4] = {0,0,0,0}; // Actual duty (0-255) for each H-bridge input

// Joystick mode state (signed speed -255..255 for each side)
int16_t joystickLeftSpeed = 0;
int16_t joystickRightSpeed = 0;
// Timestamp of last joystick command (millis) for deadman watchdog
unsigned long lastJoystickCmdMs = 0;
// Deadman interval (ms) after which motors auto-stop if no new joystick updates
const unsigned long JOYSTICK_DEADMAN_MS = 1300; // allow >1s before auto-stop; JS keepalive sends every 1s

// PWM configuration (fallback using analogWrite if LEDC not available in this build context)
// For ESP32 Arduino core normally LEDC functions exist; if not, we degrade gracefully to analogWrite.
// analogWrite on ESP32 uses a default LEDC channel internally; distinct frequency per pin not configured here.
const int PWM_MAX = 255; // 8-bit resolution target

void applyJoystickSpeeds();

// Function declarations
void initMotors();
void setMotorPin(int pin, bool state);
void stopAllMotors();
void driveForward();
void driveReverse();
void driveLeft();
void driveRight();
String getMotorStatus();

void initMotors()
{
    Serial.println("Initializing motors...");
    for (int side = 0; side < 2; side++) {
        for (int ch = 0; ch < 2; ch++) {
            int pin = MOTOR_PIN[side][ch];
            pinMode(pin, OUTPUT);
            analogWrite(pin, 0);
            motorDuty[side*2 + ch] = 0;
        }
    }

    Serial.println("Motors: Initialized");
}

void setMotorPin(int pinIndex, bool state) {
    if (pinIndex >= 0 && pinIndex < 4) {
        int side = pinIndex / 2;
        int ch = pinIndex % 2;
        int pin = MOTOR_PIN[side][ch];
        uint16_t duty = state ? PWM_MAX : 0;
        analogWrite(pin, duty);
        motorPinStates[pinIndex] = state;
        motorDuty[pinIndex] = duty;
        Serial.printf("Motors: Pin %d (GPIO %d) duty %u (%s)\n", pinIndex, pin, duty, state ? "ON" : "OFF");
    }
}

void stopAllMotors() {
    for (int side = 0; side < 2; side++) {
        for (int ch = 0; ch < 2; ch++) {
            int idx = side*2 + ch;
            analogWrite(MOTOR_PIN[side][ch], 0);
            motorDuty[idx] = 0;
            motorPinStates[idx] = false;
        }
    }
    joystickLeftSpeed = 0;
    joystickRightSpeed = 0;
    Serial.println("Motors: All stopped");
}

void driveForward() {
    // Left motor forward: Pin 0 HIGH, Pin 1 LOW
    // Right motor forward: Pin 2 HIGH, Pin 3 LOW
    analogWrite(MOTOR_PIN[0][0], PWM_MAX);
    analogWrite(MOTOR_PIN[0][1], 0);
    analogWrite(MOTOR_PIN[1][0], PWM_MAX);
    analogWrite(MOTOR_PIN[1][1], 0);
    motorDuty[0] = PWM_MAX; motorDuty[1] = 0; motorDuty[2] = PWM_MAX; motorDuty[3] = 0;
    Serial.println("Motors: Forward");
}

void driveReverse() {
    // Left motor reverse: Pin 0 LOW, Pin 1 HIGH
    // Right motor reverse: Pin 2 LOW, Pin 3 HIGH
    analogWrite(MOTOR_PIN[0][0], 0);
    analogWrite(MOTOR_PIN[0][1], PWM_MAX);
    analogWrite(MOTOR_PIN[1][0], 0);
    analogWrite(MOTOR_PIN[1][1], PWM_MAX);
    motorDuty[0] = 0; motorDuty[1] = PWM_MAX; motorDuty[2] = 0; motorDuty[3] = PWM_MAX;
    Serial.println("Motors: Reverse");
}

void driveLeft() {
    // Left motor reverse, Right motor forward
    analogWrite(MOTOR_PIN[0][0], 0);
    analogWrite(MOTOR_PIN[0][1], PWM_MAX);
    analogWrite(MOTOR_PIN[1][0], PWM_MAX);
    analogWrite(MOTOR_PIN[1][1], 0);
    motorDuty[0] = 0; motorDuty[1] = PWM_MAX; motorDuty[2] = PWM_MAX; motorDuty[3] = 0;
    Serial.println("Motors: Left turn");
}

void driveRight() {
    // Left motor forward, Right motor reverse
    analogWrite(MOTOR_PIN[0][0], PWM_MAX);
    analogWrite(MOTOR_PIN[0][1], 0);
    analogWrite(MOTOR_PIN[1][0], 0);
    analogWrite(MOTOR_PIN[1][1], PWM_MAX);
    motorDuty[0] = PWM_MAX; motorDuty[1] = 0; motorDuty[2] = 0; motorDuty[3] = PWM_MAX;
    Serial.println("Motors: Right turn");
}

String getMotorStatus() {
    JsonDocument doc;
    if (currentMode == MODE_PIN_TEST) {
        doc["mode"] = "pin_test";
        doc["summary"] = "Manual pin toggle mode";
    } else if (currentMode == MODE_SIMPLE_DRIVE) {
        doc["mode"] = "simple_drive";
        doc["summary"] = "Directional drive mode";
    } else {
        doc["mode"] = "joystick";
        doc["summary"] = "Analog joystick PWM control";
        doc["left_speed"] = joystickLeftSpeed;
        doc["right_speed"] = joystickRightSpeed;
    }
    
    JsonArray pins = doc["pins"].to<JsonArray>();
    for (int i = 0; i < 4; i++) {
        JsonObject pin = pins.add<JsonObject>();
        pin["index"] = i;
        pin["gpio"] = MOTOR_PIN[i/2][i%2];
    pin["state"] = motorDuty[i] > 0;
    pin["duty"] = motorDuty[i];
    }
    
    String result;
    serializeJson(doc, result);
    return result;
}

// Helper to constrain and map speed to PWM duties (exclusive direction)
static void writeMotorSigned(int16_t speed, int pinFwd, int pinRev) {
    speed = constrain(speed, -PWM_MAX, PWM_MAX);
    if (speed > 0) {
        analogWrite(pinRev, 0); analogWrite(pinFwd, speed);
    } else if (speed < 0) {
        analogWrite(pinFwd, 0); analogWrite(pinRev, -speed);
    } else {
        analogWrite(pinFwd, 0); analogWrite(pinRev, 0);
    }
    // Update duty tracking (we need index resolution)
    for (int i=0;i<4;i++) {
        if (MOTOR_PIN[i/2][i%2] == pinFwd) motorDuty[i] = (speed>0)? speed:0;
        if (MOTOR_PIN[i/2][i%2] == pinRev) {
            if (speed<0) motorDuty[i] = -speed; else if (speed>0) motorDuty[i] = 0; else motorDuty[i] = 0;
        }
    }
}

void applyJoystickSpeeds() {
    // Left motor: forward pin MOTOR_PIN[0][0], reverse pin MOTOR_PIN[0][1]
    writeMotorSigned(joystickLeftSpeed, MOTOR_PIN[0][0], MOTOR_PIN[0][1]);
    // Right motor: forward pin MOTOR_PIN[1][0], reverse pin MOTOR_PIN[1][1]
    writeMotorSigned(joystickRightSpeed, MOTOR_PIN[1][0], MOTOR_PIN[1][1]);
    lastJoystickCmdMs = millis();
}

// Call periodically (e.g., from main loop) to enforce joystick deadman stop
inline void joystickDeadmanCheck() {
    if (currentMode == MODE_JOYSTICK) {
        unsigned long now = millis();
        if (lastJoystickCmdMs != 0 && (now - lastJoystickCmdMs) > JOYSTICK_DEADMAN_MS) {
            if (joystickLeftSpeed != 0 || joystickRightSpeed != 0) {
                joystickLeftSpeed = 0;
                joystickRightSpeed = 0;
                applyJoystickSpeeds(); // this will refresh timestamp; reset it so next check waits again
                lastJoystickCmdMs = 0; // prevent immediate re-trigger
                Serial.println("Motors: Joystick deadman engaged -> motors stopped");
            }
        }
    }
}

// Public function for joystick control (x,y in -1..1 float mapped later externally) could be added if needed

