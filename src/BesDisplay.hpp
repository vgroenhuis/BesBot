#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "BesWifi.hpp"


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define OLED_MOSI   7
#define OLED_CLK   6
#define OLED_DC    4
#define OLED_CS    5
#define OLED_RESET 24



Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

void initDisplay() {
    Serial.println("Initializing display...");
    if(!display.begin(SSD1306_SWITCHCAPVCC)) {
        Serial.println(F("SSD1306 allocation failed"));
        for(;;); // Don't proceed, loop forever
    } else {
        Serial.println(F("SSD1306 initialized"));
    }
    delay(100);

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println("BesBot WiFi Manager");
    display.println("Initializing...");
    display.display();
}

void updateDisplay() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    
    // Title
    display.println("BesBot WiFi Status");
    
    if (manager.isConnected()) {
        // Connected - show WiFi info
        display.printf("SSID: %s\n", WiFi.SSID().c_str());
        //display.printf("IP: %s\n", WiFi.localIP().toString().c_str());
        display.printf("RSSI: %d dBm\n", WiFi.RSSI());
        
        // Show web server URL
        display.printf("http://%s\n", WiFi.localIP().toString().c_str());
    } else {
        // Not connected
        display.println("Status: Connecting...");
        display.println("Scanning networks...");
        
        // Show connection dots animation
        static int dots = 0;
        dots = (dots + 1) % 4;
        for (int i = 0; i < dots; i++) {
            display.print(".");
        }
    }
    
    display.display();
}
