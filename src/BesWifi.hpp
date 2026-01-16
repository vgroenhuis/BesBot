#pragma once

#if __has_include("../../VincentLibraryTester/include/VincentCredentials.h")
    #include "../../VincentLibraryTester/include/VincentCredentials.h"
#else
    #define WIFI_CREDENTIALS {{"ssid","pass"},{"iotroam","pass2"}}
    #define ADMIN_CREDENTIALS {"",""}
    #define ALIAS_URL ""
#endif

#include <Arduino.h>
#include "RoamingWiFiManager.h"
#include "BesMotors.hpp"
#include "BesMainPage.html.h"

RoamingWiFiManager manager;

void loopWifi() {
    manager.loop();
}


void setupWebServer() {
    // Serve main page
    manager.server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/html", BESBOT_WIFI_HTML);
    });


    // Motor control endpoints
    // /motor/mode  POST  {"mode":"pin_test"|"simple_drive"|"joystick"}
    manager.server.on("/motor/mode", HTTP_POST, [](AsyncWebServerRequest *request) {
        // If someone posts without body callback (shouldn't happen), return current mode
        JsonDocument resp;
    if (currentMode == MODE_PIN_TEST) resp["mode"] = "pin_test";
    else if (currentMode == MODE_SIMPLE_DRIVE) resp["mode"] = "simple_drive";
    else resp["mode"] = "joystick";
        String out; serializeJson(resp, out);
        request->send(200, "application/json", out);
    }, NULL, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
        static String bodyAccum;
        if (index == 0) bodyAccum = ""; // first chunk
        for (size_t i = 0; i < len; i++) bodyAccum += (char)data[i];
        if (index + len < total) return; // wait for last chunk

        JsonDocument doc;
        DeserializationError err = deserializeJson(doc, bodyAccum);
        JsonDocument resp;
        if (err) {
            resp["ok"] = false;
            resp["error"] = String("Invalid JSON: ") + err.c_str();
        } else {
            String mode = doc["mode"] | "";
            if (mode == "pin_test" || mode == "simple_drive" || mode == "joystick") {
                MotorMode newMode = MODE_PIN_TEST;
                if (mode == "simple_drive") newMode = MODE_SIMPLE_DRIVE; else if (mode == "joystick") newMode = MODE_JOYSTICK;
                if (newMode != currentMode) {
                    currentMode = newMode;
                    stopAllMotors();
                    Serial.printf("Motors: Mode changed to %s\n", mode.c_str());
                }
                resp["ok"] = true;
                resp["mode"] = mode;
            } else {
                resp["ok"] = false;
                resp["error"] = "Unsupported mode";
                resp["expected"] = "pin_test|simple_drive|joystick";
            }
        }
        String out; serializeJson(resp, out);
        request->send(200, "application/json", out);
    });

    // Joystick control: POST /motor/joystick { x: float -1..1 (turn), y: float -1..1 (forward) }
    manager.server.on("/motor/joystick", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
        static String bodyAccum; if (index == 0) bodyAccum = ""; for (size_t i=0;i<len;i++) bodyAccum += (char)data[i]; if (index + len < total) return;
        JsonDocument resp;
        if (currentMode != MODE_JOYSTICK) {
            resp["ok"] = false; resp["error"] = "Not in joystick mode"; String out; serializeJson(resp, out); request->send(200, "application/json", out); return;
        }
        if (bodyAccum.length() == 0) { resp["ok"] = false; resp["error"] = "Empty body"; String out; serializeJson(resp, out); request->send(200, "application/json", out); return; }
        JsonDocument doc; DeserializationError err = deserializeJson(doc, bodyAccum);
        if (err) { resp["ok"] = false; resp["error"] = String("Invalid JSON: ") + err.c_str(); }
        else {
            float x = doc["x"].as<float>();
            float y = doc["y"].as<float>();
            if (isnan(x) || isnan(y)) { resp["ok"] = false; resp["error"] = "NaN values"; }
            else {
                // Clamp
                if (x > 1) x = 1; if (x < -1) x = -1; if (y > 1) y = 1; if (y < -1) y = -1;
                // Differential mixing: left = y + x, right = y - x
                float left = y + x;
                float right = y - x;
                // Normalize if any magnitude exceeds 1
                float maxMag = max(fabs(left), fabs(right));
                if (maxMag > 1) { left /= maxMag; right /= maxMag; }
                // Scale to PWM 255
                joystickLeftSpeed = (int16_t)round(left * 255.0f);
                joystickRightSpeed = (int16_t)round(right * 255.0f);
                applyJoystickSpeeds();
                resp["ok"] = true;
                resp["x"] = x; resp["y"] = y;
                resp["left_speed"] = joystickLeftSpeed; resp["right_speed"] = joystickRightSpeed;
            }
        }
        String out; serializeJson(resp, out); request->send(200, "application/json", out);
    });

    manager.server.on("/motor/pin", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
        static String bodyAccum;
        if (index == 0) bodyAccum = "";
        for (size_t i = 0; i < len; i++) bodyAccum += (char)data[i];
        if (index + len < total) return; // wait until full body received

        JsonDocument resp;
        if (bodyAccum.length() == 0) {
            resp["ok"] = false; resp["error"] = "Empty body";
        } else {
            JsonDocument doc; DeserializationError err = deserializeJson(doc, bodyAccum);
            if (err) {
                resp["ok"] = false; resp["error"] = String("Invalid JSON: ") + err.c_str();
            } else {
                int pin = doc["pin"].as<int>(); String action = doc["action"].as<String>();
                if (currentMode != MODE_PIN_TEST) { resp["ok"] = false; resp["error"] = "Not in pin_test mode"; }
                else if (pin < 0 || pin > 3) { resp["ok"] = false; resp["error"] = "Pin out of range (0-3)"; }
                else if (action != "toggle" && action != "on" && action != "off") { resp["ok"] = false; resp["error"] = "Unsupported action"; }
                else {
                    bool targetState;
                    if (action == "toggle") { bool currentState = digitalRead(MOTOR_PIN[pin/2][pin%2]) == HIGH; targetState = !currentState; }
                    else targetState = (action == "on");
                    setMotorPin(pin, targetState);
                    resp["ok"] = true; resp["pin"] = pin; resp["state"] = targetState; resp["action"] = action;
                }
            }
        }
        String out; serializeJson(resp, out); request->send(200, "application/json", out);
    });

    manager.server.on("/motor/drive", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
        static String bodyAccum; if (index == 0) bodyAccum = ""; for (size_t i=0;i<len;i++) bodyAccum += (char)data[i]; if (index + len < total) return;
        JsonDocument resp;
        if (bodyAccum.length() == 0) {
            resp["ok"] = false; resp["error"] = "Empty body";
        } else {
            JsonDocument doc; DeserializationError err = deserializeJson(doc, bodyAccum);
            if (err) { resp["ok"] = false; resp["error"] = String("Invalid JSON: ") + err.c_str(); }
            else {
                String direction = doc["direction"].as<String>();
                if (currentMode != MODE_SIMPLE_DRIVE) { resp["ok"] = false; resp["error"] = "Not in simple_drive mode"; }
                else {
                    bool recognized = true;
                    if (direction == "forward") driveForward();
                    else if (direction == "reverse") driveReverse();
                    else if (direction == "left") driveLeft();
                    else if (direction == "right") driveRight();
                    else if (direction == "stop") stopAllMotors();
                    else recognized = false;
                    if (recognized) { resp["ok"] = true; resp["direction"] = direction; }
                    else { resp["ok"] = false; resp["error"] = "Unsupported direction"; resp["expected"] = "forward|reverse|left|right|stop"; }
                }
            }
        }
        String out; serializeJson(resp, out); request->send(200, "application/json", out);
    });

    manager.server.on("/motor/status", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "application/json", getMotorStatus());
    });
}


void initWifi() {
    manager.init(WIFI_CREDENTIALS, ADMIN_CREDENTIALS, ALIAS_URL);
    setupWebServer();
}








// WiFi Configuration Structure
/*
struct WiFiNetwork {
    const char* ssid;
    const char* password;
};

// Define known networks (preferring 5GHz)
const WiFiNetwork knownNetworks[] = {
    {"VincentS21FE", "brgf9069"},
    {"Mojca_Vincent_Reelaan3", "batterry"},
    {"iotroam", "besbotroam"}
};
const int numKnownNetworks = sizeof(knownNetworks) / sizeof(knownNetworks[0]);

// Web server instance
AsyncWebServer server(80);

// Global variables for WiFi status
unsigned long wifiConnectedTime = 0;
unsigned long lastScanTime = 0;
unsigned long lastAutoScanTime = 0;
const unsigned long autoScanInterval = 15000; // Auto-scan every 15 seconds
bool scanInProgress = false;
bool connectionRequested = false; // Flag for connection request from web interface
bool autoScanEnabled = false; // Setting to enable/disable periodic network scanning
int numNetworks = 0;
String lastScannedNetworks = "[]"; // JSON string of scanned networks
Preferences wifiPrefs;            // NVS preferences for persistence
String savedBSSID = "";          // Last successfully used BSSID (persisted)
String savedSSID = "";           // Last successfully connected SSID
int savedChannel = 0;             // Last channel used

// Forward declarations for new persistence helpers
bool connectToSavedBSSID();                // Uses cached scan list
bool connectDirectSaved();                 // Uses persisted SSID/BSSID without scan
void saveCurrentBSSID();
void loadSavedBSSID();
bool parseBSSID(const String &macStr, uint8_t *outBytes);

// Function declarations
void initWifi();
void loopWifi();
void connectToStrongestKnownNetwork();
void handleWiFiEvent(WiFiEvent_t event);
void setupWebServer();
String getWiFiStatus();
String getVisibleNetworks();
void scanNetworksAsync();
*/

    /*
    // Set WiFi mode and register event handler
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    // Attempt to set 5GHz band preference (if supported by ESP32-C5)
    WiFi.setBandMode(WIFI_BAND_MODE_5G_ONLY);
    WiFi.onEvent(handleWiFiEvent);
    //esp_wifi_get_protocols(WIFI_IF_STA, NULL);

    
    // Enable 5GHz band preference (if supported by ESP32-C5)
    // Note: ESP32-C5 supports 2.4GHz by default, check if 5GHz is available
    
    Serial.println("WiFi: Initializing...");
    // Print the ESP32 station MAC address before attempting any connection
    // This helps identify the device on the network even if connection fails.
    String stationMac = WiFi.macAddress();
    Serial.printf("WiFi: Station MAC (STA): %s\n", stationMac.c_str());
    // If you later enable AP mode you could also print the softAP MAC via WiFi.softAPmacAddress().

    // Open preferences namespace and load previously saved connection info
    wifiPrefs.begin("wifi", false);
    //loadSavedBSSID();

    bool fastPathUsed = false;
    bool connected = false;
    if (savedBSSID.length() > 0 && savedSSID.length() > 0) {
        Serial.printf("WiFi: Attempting fast reconnect without initial scan (SSID=%s, BSSID=%s, channel=%d) ...\n", savedSSID.c_str(), savedBSSID.c_str(), savedChannel);
        fastPathUsed = true;
        connected = connectDirectSaved();
    }

    if (!connected) {
        if (fastPathUsed) {
            Serial.println("WiFi: Fast reconnect failed, performing initial scan...");
        } else {
            Serial.println("WiFi: No saved connection info; performing initial scan...");
        }
        rgbLedWrite(RGB_BUILTIN, 10, 5, 0); // Orange for scanning
        int n = WiFi.scanNetworks();
        if (n >= 0) {
            JsonDocument doc;
            JsonArray networks = doc["networks"].to<JsonArray>();
            for (int i = 0; i < n; i++) {
                JsonObject network = networks.add<JsonObject>();
                network["ssid"] = WiFi.SSID(i);
                network["bssid"] = WiFi.BSSIDstr(i);
                network["rssi"] = WiFi.RSSI(i);
                network["channel"] = WiFi.channel(i);
                network["encryption"] = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "Open" : "Encrypted";
                Serial.printf("WiFi: Found network: %s (RSSI: %d)\n", WiFi.SSID(i).c_str(), WiFi.RSSI(i));
            }
            serializeJson(doc, lastScannedNetworks);
            numNetworks = n;
            Serial.printf("WiFi: Initial scan completed, found %d networks\n", n);
        } else {
            Serial.println("WiFi: Initial scan failed");
        }
        WiFi.scanDelete();
        lastAutoScanTime = millis();

        // Try targeted BSSID using freshly built cache (verifies still present)
        if (!connected && savedBSSID.length() > 0) {
            Serial.println("WiFi: Trying targeted reconnect using scanned cache...");
            connected = connectToSavedBSSID();
        }
        // Fallback to strongest known if not connected yet
        if (!connected) {
            connectToStrongestKnownNetwork();
        }
    } else {
        // Fast path success: schedule a background async scan so UI sees networks shortly
        Serial.println("WiFi: Fast reconnect succeeded; scheduling background async scan for network list.");
        lastAutoScanTime = millis();
        // Kick off an async scan (non-blocking) after short delay
        scanNetworksAsync();
    }

    // Setup web server after connection attempts
    setupWebServer();
    */



    /*
    // Handle connection request from web interface
    if (connectionRequested) {
        connectionRequested = false;
        Serial.println("WiFi: Processing connection request from web interface...");
        connectToStrongestKnownNetwork();
    }
    
    // Handle WiFi reconnection if disconnected
    if (WiFi.status() != WL_CONNECTED && millis() - lastScanTime > 30000) {
        Serial.println("WiFi: Attempting reconnection...");
        connectToStrongestKnownNetwork();
    }
    
    // Automatic network scanning every x seconds (if enabled)
    if (autoScanEnabled && !scanInProgress && millis() - lastAutoScanTime > autoScanInterval) {
        Serial.println("WiFi: Starting automatic network scan...");
        scanNetworksAsync();
        lastAutoScanTime = millis();
    }
    
    // Handle async scan completion
    if (scanInProgress && WiFi.scanComplete() != WIFI_SCAN_RUNNING) {
        scanInProgress = false;
        int n = WiFi.scanComplete();
        if (n >= 0) {
            JsonDocument doc;
            JsonArray networks = doc["networks"].to<JsonArray>();
            
            for (int i = 0; i < n; i++) {
                JsonObject network = networks.add<JsonObject>();
                network["ssid"] = WiFi.SSID(i);
                network["bssid"] = WiFi.BSSIDstr(i);
                network["rssi"] = WiFi.RSSI(i);
                network["channel"] = WiFi.channel(i);
                network["encryption"] = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "Open" : "Encrypted";

                Serial.printf("WiFi: Found network: %s (RSSI: %d)\n", WiFi.SSID(i).c_str(), WiFi.RSSI(i));
            }
            
            serializeJson(doc, lastScannedNetworks);
            numNetworks = n;
            Serial.printf("WiFi: Auto-scan completed, found %d networks\n", n);
        }
        WiFi.scanDelete();
    }
        */

/*
void connectToStrongestKnownNetwork() {
    Serial.println("WiFi: Connecting to strongest known network using cached data...");
    
    // Set LED to orange while connecting
    rgbLedWrite(RGB_BUILTIN, 10, 5, 0); // Orange for connecting
    
    // Parse cached network data
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, lastScannedNetworks);
    
    if (error) {
        Serial.println("WiFi: No cached network data available, skipping connection attempt");
        rgbLedWrite(RGB_BUILTIN, 10, 0, 0); // Red for no data
        return;
    }
    
    JsonArray networks = doc["networks"];
    int bestRSSI = -100;
    int bestKnownIndex = -1;
    String bestSSID = "";
    
    // Find the strongest known network from cached data
    for (JsonVariant networkVar : networks) {
        JsonObject network = networkVar.as<JsonObject>();
        String scannedSSID = network["ssid"].as<String>();
        int rssi = network["rssi"].as<int>();
        
        // Check if this is a known network
        for (int j = 0; j < numKnownNetworks; j++) {
            if (scannedSSID.equals(knownNetworks[j].ssid)) {
                if (rssi > bestRSSI) {
                    bestRSSI = rssi;
                    bestKnownIndex = j;
                    bestSSID = scannedSSID;
                }
                break;
            }
        }
    }
    
    if (bestKnownIndex >= 0) {
        Serial.printf("WiFi: Connecting to %s (RSSI: %d) from cache\n", 
                     knownNetworks[bestKnownIndex].ssid, bestRSSI);
        
        WiFi.begin(knownNetworks[bestKnownIndex].ssid, knownNetworks[bestKnownIndex].password);
        
        // Wait for connection (with timeout)
        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED && attempts < 20) {
            delay(500);
            Serial.print(".");
            attempts++;
        }
        
        if (WiFi.status() == WL_CONNECTED) {
            wifiConnectedTime = millis();
            Serial.println("\nWiFi: Connected successfully!");
            Serial.printf("IP Address: %s\n", WiFi.localIP().toString().c_str());
            // Print device MAC and connected BSSID info
            Serial.printf("Station MAC: %s\n", WiFi.macAddress().c_str());
            Serial.printf("AP BSSID: %s  Channel: %d  RSSI: %d dBm\n", WiFi.BSSIDstr().c_str(), WiFi.channel(), WiFi.RSSI());
            // Set LED to green when connected
            rgbLedWrite(RGB_BUILTIN, 0, 10, 0); // Green for connected
        } else {
            Serial.println("\nWiFi: Connection failed!");
            // Keep LED orange for failed connection
            rgbLedWrite(RGB_BUILTIN, 10, 5, 0); // Orange for connection failed
        }
    } else {
        Serial.println("WiFi: No known networks found in cache!");
        // Set LED to red when no known networks found
        rgbLedWrite(RGB_BUILTIN, 10, 0, 0); // Red for no networks found
    }
    
    lastScanTime = millis();
}

void handleWiFiEvent(WiFiEvent_t event) {
    switch (event) {
        case ARDUINO_EVENT_WIFI_STA_CONNECTED:
            Serial.println("WiFi: Connected to access point");
            wifiConnectedTime = millis();
            break;
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            Serial.println("WiFi: Disconnected from access point");
            break;
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            Serial.printf("WiFi: Got IP address: %s\n", WiFi.localIP().toString().c_str());
            Serial.printf("WiFi: Station MAC: %s\n", WiFi.macAddress().c_str());
            Serial.printf("WiFi: Connected BSSID: %s  Channel: %d  RSSI: %d dBm\n", WiFi.BSSIDstr().c_str(), WiFi.channel(), WiFi.RSSI());
            // Persist connection info
            saveCurrentBSSID();
            break;
        default:
            break;
    }
}
*/



    // WiFi settings endpoints
    /*
    manager.server.on("/wifi/autoscan", HTTP_POST, [](AsyncWebServerRequest *request) {
        request->send(200, "application/json", "{\"message\":\"Auto-scan setting updated\"}");
    }, NULL, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
        String body = "";
        for (size_t i = 0; i < len; i++) {
            body += (char)data[i];
        }
        JsonDocument doc;
        deserializeJson(doc, body);
        bool enabled = doc["enabled"];
        
        autoScanEnabled = enabled;
        Serial.printf("WiFi: Auto-scan %s\n", enabled ? "enabled" : "disabled");
        
        request->send(200, "application/json", "{\"message\":\"Auto-scan setting updated\"}");
    });
    */

    /*
    server.on("/wifi/settings", HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;
        doc["autoScanEnabled"] = autoScanEnabled;
        doc["autoScanInterval"] = autoScanInterval;
        
        String result;
        serializeJson(doc, result);
        request->send(200, "application/json", result);
    });
    */
    //server.begin();
    //Serial.println("Web server started on http://" + WiFi.localIP().toString());


/*
String getWiFiStatus() {
    JsonDocument doc;
    
    doc["connected"] = (WiFi.status() == WL_CONNECTED);
    doc["ssid"] = WiFi.SSID();
    doc["bssid"] = WiFi.BSSIDstr();
    doc["ip"] = WiFi.localIP().toString();
    doc["rssi"] = WiFi.RSSI();
    doc["mac"] = WiFi.macAddress();
    doc["channel"] = WiFi.channel();
    doc["saved_bssid"] = savedBSSID; // expose persisted BSSID
    doc["saved_ssid"] = savedSSID;
    doc["saved_channel"] = savedChannel;
    
    // Calculate uptime
    if (wifiConnectedTime > 0 && WiFi.status() == WL_CONNECTED) {
        unsigned long uptimeMs = millis() - wifiConnectedTime;
        unsigned long uptimeSeconds = uptimeMs / 1000;
        unsigned long hours = uptimeSeconds / 3600;
        unsigned long minutes = (uptimeSeconds % 3600) / 60;
        unsigned long seconds = uptimeSeconds % 60;
        
        char uptimeStr[32];
        sprintf(uptimeStr, "%02luh %02lum %02lus", hours, minutes, seconds);
        doc["uptime"] = String(uptimeStr);
    } else {
        doc["uptime"] = "Not connected";
    }
    
    String result;
    serializeJson(doc, result);
    return result;
}

void scanNetworksAsync() {
    if (!scanInProgress) {
        scanInProgress = true;
        WiFi.scanNetworks(true); // Async scan
        Serial.println("WiFi: Starting async network scan...");
    }
}

// ========================== Persistence Helpers ===========================

void loadSavedBSSID() {
    savedBSSID = wifiPrefs.getString("last_bssid", "");
    savedSSID = wifiPrefs.getString("last_ssid", "");
    savedChannel = wifiPrefs.getInt("last_channel", 0);
    if (savedBSSID.length() > 0) {
        Serial.printf("WiFi: Loaded saved connection: SSID=%s BSSID=%s channel=%d\n", savedSSID.c_str(), savedBSSID.c_str(), savedChannel);
    } else {
        Serial.println("WiFi: No saved connection info in NVS");
    }
}

void saveCurrentBSSID() {
    if (WiFi.status() != WL_CONNECTED) return;
    const uint8_t *bssid = WiFi.BSSID();
    if (!bssid) return;
    char buf[18];
    sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X", bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5]);
    String current = String(buf);
    String currentSSID = WiFi.SSID();
    int currentChannel = WiFi.channel();
    bool changed = (current != savedBSSID) || (currentSSID != savedSSID) || (currentChannel != savedChannel);
    if (changed) {
        savedBSSID = current;
        savedSSID = currentSSID;
        savedChannel = currentChannel;
        wifiPrefs.putString("last_bssid", savedBSSID);
        wifiPrefs.putString("last_ssid", savedSSID);
        wifiPrefs.putInt("last_channel", savedChannel);
        Serial.printf("WiFi: Saved connection info: SSID=%s BSSID=%s channel=%d\n", savedSSID.c_str(), savedBSSID.c_str(), savedChannel);
    }
}

bool parseBSSID(const String &macStr, uint8_t *outBytes) {
    // Expect format AA:BB:CC:DD:EE:FF (case-insensitive)
    if (macStr.length() != 17) return false;
    int values[6];
    if (sscanf(macStr.c_str(), "%x:%x:%x:%x:%x:%x", &values[0], &values[1], &values[2], &values[3], &values[4], &values[5]) != 6) return false;
    for (int i = 0; i < 6; ++i) outBytes[i] = (uint8_t)values[i];
    return true;
}

bool connectToSavedBSSID() {
    if (savedBSSID.length() == 0) return false;

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, lastScannedNetworks);
    if (err) {
        Serial.println("WiFi: Failed to parse cached networks for saved BSSID attempt");
        return false;
    }
    JsonArray networks = doc["networks"];
    String targetSSID = "";
    int targetChannel = 0;
    bool known = false;

    for (JsonVariant v : networks) {
        JsonObject o = v.as<JsonObject>();
        String b = o["bssid"].as<String>();
        if (b.equalsIgnoreCase(savedBSSID)) {
            targetSSID = o["ssid"].as<String>();
            targetChannel = o["channel"].as<int>();
            // verify SSID is known
            for (int i = 0; i < numKnownNetworks; ++i) {
                if (targetSSID.equals(knownNetworks[i].ssid)) {
                    known = true;
                    break;
                }
            }
            break;
        }
    }

    if (!known || targetSSID.length() == 0) {
        Serial.println("WiFi: Saved BSSID not in current scan or SSID not known, skipping targeted connect");
        return false;
    }

    uint8_t bssidBytes[6];
    if (!parseBSSID(savedBSSID, bssidBytes)) {
        Serial.println("WiFi: Failed to parse saved BSSID string, skipping targeted connect");
        return false;
    }

    // Find credentials
    const char *password = nullptr;
    for (int i = 0; i < numKnownNetworks; ++i) {
        if (targetSSID.equals(knownNetworks[i].ssid)) {
            password = knownNetworks[i].password;
            break;
        }
    }
    if (!password) return false;

    Serial.printf("WiFi: Attempting connection to saved BSSID %s (SSID=%s, channel=%d)\n", savedBSSID.c_str(), targetSSID.c_str(), targetChannel);
    rgbLedWrite(RGB_BUILTIN, 10, 5, 0); // Orange while connecting

    WiFi.begin(targetSSID.c_str(), password, targetChannel, bssidBytes, true);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
        wifiConnectedTime = millis();
        Serial.println("WiFi: Connected via saved BSSID!");
        Serial.printf("IP Address: %s\n", WiFi.localIP().toString().c_str());
        Serial.printf("Station MAC: %s\n", WiFi.macAddress().c_str());
        Serial.printf("AP BSSID: %s  Channel: %d  RSSI: %d dBm\n", WiFi.BSSIDstr().c_str(), WiFi.channel(), WiFi.RSSI());
        rgbLedWrite(RGB_BUILTIN, 0, 10, 0); // Green
        return true;
    } else {
        Serial.println("WiFi: Targeted BSSID connect failed");
        return false;
    }
}

bool connectDirectSaved() {
    if (savedBSSID.length() == 0 || savedSSID.length() == 0) return false;
    // Validate SSID is in knownNetworks (for password)
    const char *password = nullptr;
    for (int i = 0; i < numKnownNetworks; ++i) {
        if (savedSSID.equals(knownNetworks[i].ssid)) {
            password = knownNetworks[i].password;
            break;
        }
    }
    if (!password) {
        Serial.println("WiFi: Saved SSID not in knownNetworks; cannot fast reconnect");
        return false;
    }
    uint8_t bssidBytes[6];
    if (!parseBSSID(savedBSSID, bssidBytes)) {
        Serial.println("WiFi: Saved BSSID parse failed; cannot fast reconnect");
        return false;
    }
    rgbLedWrite(RGB_BUILTIN, 10, 5, 0);
    // If savedChannel seems invalid (0) we pass 0 to let stack scan; else use saved channel for speed.
    int ch = (savedChannel > 0 && savedChannel <= 165) ? savedChannel : 0;
    WiFi.begin(savedSSID.c_str(), password, ch, bssidBytes, true);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(250);
        attempts++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        wifiConnectedTime = millis();
        Serial.println("WiFi: Fast reconnect succeeded (no initial scan)");
        Serial.printf("IP Address: %s\n", WiFi.localIP().toString().c_str());
        Serial.printf("Station MAC: %s\n", WiFi.macAddress().c_str());
        Serial.printf("AP BSSID: %s  Channel: %d  RSSI: %d dBm\n", WiFi.BSSIDstr().c_str(), WiFi.channel(), WiFi.RSSI());
        rgbLedWrite(RGB_BUILTIN, 0, 10, 0);
        return true;
    }
    Serial.println("WiFi: Fast reconnect failed");
    return false;
}
*/
