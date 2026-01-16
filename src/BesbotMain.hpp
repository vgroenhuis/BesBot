/*
    BesBot: a SmartXP robot with ESP32-C5
    Documentation (Vincent): https://universiteittwente-my.sharepoint.com/:o:/r/personal/v_groenhuis_utwente_nl/Documents/Vincent%20@%20University%20of%20Twente?d=wa924ed023c5c4c2e93aa224c973b1cfd&csf=1&web=1&e=HIOpmZ
*/


#include <Arduino.h>

#include "BesWifi.hpp"
#include "BesMotors.hpp"
#include "BesDisplay.hpp"


unsigned long lastDisplayUpdate = 0;
const unsigned long displayUpdateInterval = 2000; // Update display every 2 seconds


void setup() {
    Serial.begin(115200);
    Serial.println("BesBot starting...");
    //rgbLedWrite(RGB_BUILTIN, 1,0,0); // dark red
    initDisplay();
    initMotors();
    initWifi();
}

void loop() {
    // Handle WiFi operations
    loopWifi();
    
    // Update display periodically
    if (millis() - lastDisplayUpdate >= displayUpdateInterval) {
        updateDisplay();
        lastDisplayUpdate = millis();
    }
    
    /*
    // Update LED status based on WiFi connection
    if (WiFi.status() == WL_CONNECTED) {
        rgbLedWrite(RGB_BUILTIN, 0, 10, 0); // Green for connected
    } else {
        rgbLedWrite(RGB_BUILTIN, 10, 5, 0);  // Orange for connecting
    }
    */
    // Joystick deadman watchdog
    joystickDeadmanCheck();
    
    // Small delay to prevent excessive CPU usage
    delay(1);
}

