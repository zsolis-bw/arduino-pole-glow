#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "esp_log.h"
#include <unordered_map>
#include <string>
#include <iostream>

// Pin and hardware definitions
#define DATA_PIN 17
#define NUM_LEDS 50
#define BUTTON_PINS {0, 18} // Array of GPIO pins to use for buttons
#define DEBUG_MODE true

// List debug functions and their enabled status
std::unordered_map<std::string, bool> debugFunctions = {
    {"GPS_SPEED", false},
    {"GPS_LOC", false},
    {"GPS_ELEV", false},
    {"GPS_SATS", false},
    {"GYRO_XYZ", false},
    {"GYRO_ACCEL", false},
    {"MEMORY_SYNC", false}
};

const int buttonPins[] = BUTTON_PINS;
const int numberOfButtons = sizeof(buttonPins) / sizeof(buttonPins[0]);

// STEMMA QT Pins (and Baud Rate for GPS)
#define RXPin 41  // GPS TX -> ESP RX
#define TXPin 40  // GPS RX -> ESP TX
#define GPSBaud 38400

// GPS and MPU objects
TinyGPSPlus gps;
HardwareSerial gpsSerial(1); // Use Serial1 for GPS
Adafruit_MPU6050 mpu;

// GPS data variables
float currentSpeed = 0;
float currentLatitude = 0.0;
float currentLongitude = 0.0;
float currentElevation = 0.0;
unsigned int gpsSatellites = 0;
float gpsHDOP = 0.0;

// MPU data variables
float currentDirection = 0;
float accelX = 0.0;
float accelY = 0.0;
float accelZ = 0.0;
float gyroX = 0.0;
float gyroY = 0.0;
float gyroZ = 0.0;

// Other variables
unsigned long lastGPSUpdate = 0;
const unsigned long gpsUpdateInterval = 1000; // Update GPS every 1000 ms
uint8_t mode = 0;
uint16_t hue = 0;
bool hasGPS = false;
bool hasMPU = false;
bool isMaster = false;

// NeoPixel and ESP-NOW structures
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, DATA_PIN, NEO_GRB + NEO_KHZ800);

#ifdef PIN_NEOPIXEL
#define BUILTIN_LED_PIN PIN_NEOPIXEL
Adafruit_NeoPixel builtInLED = Adafruit_NeoPixel(1, BUILTIN_LED_PIN, NEO_GRB + NEO_KHZ800);
#endif

typedef struct __attribute__((packed)) {
    uint16_t currentSpeed;          // Scaled to tenths of mph
    int16_t currentDirection;       // Scaled to tenths of degrees
    int16_t currentElevation;       // Scaled to meters
} GPSData;

typedef struct __attribute__((packed)) {
    int16_t accelX;          // Scaled to milli-g
    int16_t accelY;          // Scaled to milli-g
    int16_t accelZ;          // Scaled to milli-g
    int16_t gyroX;           // Scaled to tenths of degrees/sec
    int16_t gyroY;           // Scaled to tenths of degrees/sec
    int16_t gyroZ;           // Scaled to tenths of degrees/sec
} MPUData;

typedef struct __attribute__((packed)) {
    GPSData gpsData;         // GPS-specific data
    MPUData mpuData;         // MPU-specific data
} SyncData;

SyncData syncData;

// Array of peer MAC addresses (replace with actual MAC addresses of devices)
uint8_t macAddresses[][6] = {
    {0xD4, 0xF9, 0x8D, 0x66, 0x12, 0xCA}, // Replace with your first ESP32 MAC
    {0xF4, 0x12, 0xFA, 0x59, 0x5B, 0x30}  // Replace with your second ESP32 MAC
};

uint8_t selfMac[6];      // This device's MAC address
uint8_t oppositeMac[6];  // The opposite device's MAC address

esp_now_peer_info_t peerInfo;

void initializePeers() {
    for (int i = 0; i < sizeof(macAddresses) / sizeof(macAddresses[0]); i++) {
        memcpy(peerInfo.peer_addr, macAddresses[i], 6);
        peerInfo.channel = 0; // Use default channel
        peerInfo.encrypt = false; // No encryption for now

        if (esp_now_add_peer(&peerInfo) != ESP_OK) {
            Serial.printf("Failed to add peer: %02X:%02X:%02X:%02X:%02X:%02X\n",
                          macAddresses[i][0], macAddresses[i][1], macAddresses[i][2],
                          macAddresses[i][3], macAddresses[i][4], macAddresses[i][5]);
        } else {
            Serial.printf("Added peer: %02X:%02X:%02X:%02X:%02X:%02X\n",
                          macAddresses[i][0], macAddresses[i][1], macAddresses[i][2],
                          macAddresses[i][3], macAddresses[i][4], macAddresses[i][5]);
        }
    }
}

// Forward declarations
void debugOutput();
bool isDebugFunctionEnabled(const std::string& functionName);
void readGPSData();
void readGyroData();
void initializeGPS();
void initializeMPU();
void applyEffectToStrip();
void setLEDColorForMode(uint8_t mode);
void scrollingRainbow();
void directionalStrobe();
void cometTail();
void twinkleBurst();
void breathingEffect();
void gyroRainbowEffect();
void sendData();
void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len);
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void handleModeSync(const uint8_t *mac_addr, const uint8_t *incomingData, int len);
void syncMode();
bool modeRequiresGPS();
bool modeRequiresMPU();

void setup() {
    Serial.begin(115200);
    //sensor setup
    initializeGPS();
    initializeMPU();
    WiFi.mode(WIFI_STA); // Set WiFi to station mode
    esp_read_mac(selfMac, ESP_MAC_WIFI_STA); // Get this device's MAC address

    // Determine the opposite MAC address
    for (int i = 0; i < sizeof(macAddresses) / sizeof(macAddresses[0]); i++) {
        if (memcmp(selfMac, macAddresses[i], 6) != 0) { // If not this device's MAC
            memcpy(oppositeMac, macAddresses[i], 6);   // Set as opposite MAC
        }
    }

    Serial.printf("Self MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", 
                  selfMac[0], selfMac[1], selfMac[2], 
                  selfMac[3], selfMac[4], selfMac[5]);
    Serial.printf("Opposite MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", 
                  oppositeMac[0], oppositeMac[1], oppositeMac[2], 
                  oppositeMac[3], oppositeMac[4], oppositeMac[5]);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);

    // Add opposite peer
    memcpy(peerInfo.peer_addr, oppositeMac, 6);
    peerInfo.channel = 0; // Default channel
    peerInfo.encrypt = false; // No encryption
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add opposite peer.");
    } else {
        Serial.println("Opposite peer added successfully.");
    }
}

void loop() {
    // Check all button pins
    for (int i = 0; i < numberOfButtons; i++) {
        if (digitalRead(buttonPins[i]) == LOW) {
            delay(50); // Simple debounce
            while (digitalRead(buttonPins[i]) == LOW); // Wait for release
            mode = (mode + 1) % 6; // Change mode
            if (modeRequiresGPS() && hasGPS) {
                isMaster = true;
            } else if (modeRequiresMPU() && hasMPU) {
                isMaster = true;
            } else {
                isMaster = false;
            }
            syncMode();
            delay(200); // Debounce delay
        }
    }

    // Process GPS if available
    if (hasGPS) {
        readGPSData();
    }

    // Process MPU if available
    if (hasMPU) {
        readGyroData();
    }

    // Call sendData 
    sendData();

    // Debug output
    if (DEBUG_MODE) {
        debugOutput();
    }
    // Apply LED effects
    delay(20);
    applyEffectToStrip();
}

void sendData() {
    if (hasGPS) {
        GPSData gpsData = { 
            .currentSpeed = static_cast<uint16_t>(currentSpeed * 10), 
            .currentDirection = static_cast<int16_t>(currentDirection * 10), 
            .currentElevation = static_cast<int16_t>(currentElevation) 
        };
        esp_err_t result = esp_now_send(oppositeMac, (uint8_t *)&gpsData, sizeof(gpsData));
        if (result != ESP_OK && isDebugFunctionEnabled("MEMORY_SYNC")) {
            Serial.println("GPS send failed.");
        } else if(isDebugFunctionEnabled("MEMORY_SYNC")) {
            Serial.println("GPS data sent successfully.");
        }
    }

    if (hasMPU) {
        MPUData mpuData = { 
            .accelX = static_cast<int16_t>(accelX * 1000), 
            .accelY = static_cast<int16_t>(accelY * 1000), 
            .accelZ = static_cast<int16_t>(accelZ * 1000),
            .gyroX = static_cast<int16_t>(gyroX * 10), 
            .gyroY = static_cast<int16_t>(gyroY * 10), 
            .gyroZ = static_cast<int16_t>(gyroZ * 10) 
        };
        esp_err_t result = esp_now_send(oppositeMac, (uint8_t *)&mpuData, sizeof(mpuData));
        if (result != ESP_OK && isDebugFunctionEnabled("MEMORY_SYNC")) {
            Serial.println("MPU send failed.");
        } else if (isDebugFunctionEnabled("MEMORY_SYNC")) {
            Serial.println("MPU data sent successfully.");
        }
    }
}

// --- ESP-NOW Callbacks ---
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if(isDebugFunctionEnabled("MEMORY_SYNC")){
        Serial.print("Last Packet Send Status: ");
        Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
    }
}

void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
    if(isDebugFunctionEnabled("MEMORY_SYNC")){
        Serial.printf("Received data from: %02X:%02X:%02X:%02X:%02X:%02X, Length: %d\n",
                    mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5], len);
    }

    if (memcmp(mac_addr, oppositeMac, 6) == 0) {
        // Process data only if it came from the opposite MAC
        handleModeSync(mac_addr, incomingData, len);
        if (len == sizeof(GPSData)) {
            GPSData *receivedGPS = (GPSData *)incomingData;
            syncData.gpsData = *receivedGPS;
            if(isDebugFunctionEnabled("MEMORY_SYNC")){
                Serial.println("Received GPS Data");
            }
        }

        if (len == sizeof(MPUData)) {
            MPUData *receivedMPU = (MPUData *)incomingData;
            syncData.mpuData = *receivedMPU;
            if(isDebugFunctionEnabled("MEMORY_SYNC")){
                Serial.println("Received MPU Data");
            }
        }
    } else if (isDebugFunctionEnabled("MEMORY_SYNC")) {
            Serial.println("Ignored data from an unknown device.");      
    }
}

void syncMode() {
    // Sends the current mode to the opposite device
    esp_err_t result = esp_now_send(oppositeMac, &mode, sizeof(mode));
    if (result != ESP_OK) {
        Serial.println("Failed to send mode sync.");
    } else {
        Serial.println("Mode sync sent successfully.");
    }
}

void handleModeSync(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
    // Handles received mode updates
    if (len == sizeof(mode)) {
        uint8_t receivedMode;
        memcpy(&receivedMode, incomingData, sizeof(receivedMode));
        
        if (memcmp(mac_addr, oppositeMac, 6) == 0) {
            mode = receivedMode;
            Serial.printf("Mode synchronized to: %d\n", mode);
            setLEDColorForMode(mode); // Update LED state for the new mode
        }
    }
}

// helper funcs
bool isDebugFunctionEnabled(const std::string& functionName) {
    auto it = debugFunctions.find(functionName);
    if (it != debugFunctions.end()) {
        return it->second; // Return whether the function is enabled
    }
    return false; // Function not found
}

// --- Debugging ---
void debugOutput() {
    static unsigned long lastDebugTime = 0;
    const unsigned long debugInterval = 2000; // Interval in milliseconds (e.g., 2000ms = 2 seconds)

    if (millis() - lastDebugTime < debugInterval) {
        return; // Skip debug output if interval has not elapsed
    } 

    lastDebugTime = millis(); // Update the last debug time

    if(hasGPS){
        if (isDebugFunctionEnabled("GPS_SPEED")) {
            Serial.print("Speed (mph): ");
            Serial.println(currentSpeed);
        } 
        if (isDebugFunctionEnabled("GPS_LOC")) {
            Serial.print("Latitude: ");
            Serial.println(currentLatitude, 6);
            Serial.print("Longitude: ");
            Serial.println(currentLongitude, 6);
        } 
        if (isDebugFunctionEnabled("GPS_ELEV")) {
            Serial.print("Elevation (meters): ");
            Serial.println(currentElevation);
        } 
        if (isDebugFunctionEnabled("GPS_SATS")) {
            Serial.print("Satellites: ");
            Serial.println(gpsSatellites);
        }
    }
    // gyro functions
    if(hasMPU){
        if (isDebugFunctionEnabled("GYRO_XYZ")) {
            sensors_event_t a, g, temp;
            mpu.getEvent(&a, &g, &temp);
            Serial.print("Gyro X: ");
            Serial.print(g.gyro.x);
            Serial.print(" rad/s, Y: ");
            Serial.print(g.gyro.y);
            Serial.print(" rad/s, Z: ");
            Serial.println(g.gyro.z);
        } 
        if (isDebugFunctionEnabled("GYRO_ACCEL")) {
            sensors_event_t a, g, temp;
            mpu.getEvent(&a, &g, &temp);
            Serial.print("Accel X: ");
            Serial.print(a.acceleration.x);
            Serial.print(", Y: ");
            Serial.print(a.acceleration.y);
            Serial.print(", Z: ");
            Serial.println(a.acceleration.z);
        } 
    }
}

// --- GPS ---
void readGPSData() {
    bool dataUpdated = false; // Flag to track if any GPS data has been updated

    while (gpsSerial.available() > 0) {
        char c = gpsSerial.read();
        gps.encode(c); // Feed the GPS library with the received data

        // Check and update individual data points
        if (gps.location.isUpdated()) {
            currentLatitude = gps.location.lat();
            currentLongitude = gps.location.lng();
            dataUpdated = true;
        }
        if (gps.altitude.isUpdated()) {
            currentElevation = gps.altitude.meters();
            dataUpdated = true;
        }
        if (gps.speed.isUpdated()) {
            currentSpeed = gps.speed.mph() > 0.5 ? gps.speed.mph() : 0.0;
            dataUpdated = true;
        }
        if (gps.satellites.isUpdated()) {
            gpsSatellites = gps.satellites.value();
            dataUpdated = true;
        }
        if (gps.hdop.isUpdated()) {
            gpsHDOP = gps.hdop.value();
            dataUpdated = true;
        }
    }

    // Output default values if no data has been updated
    if (!dataUpdated) {
        currentLatitude = 0.0;
        currentLongitude = 0.0;
        currentElevation = 0.0;
        currentSpeed = 0.0;
        gpsSatellites = 0;
        gpsHDOP = 0.0;
    }
}

void initializeGPS() {
    Serial.println("Initializing GPS...");
    hasGPS = false; // Assume GPS is not present initially

    gpsSerial.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);

    unsigned long startTime = millis();
    gpsSerial.flush(); // Clear any residual data in the serial buffer

    while (millis() - startTime < 3000) { // 3-second timeout
        if (gpsSerial.available() > 0) {
            char c = gpsSerial.read();
            gps.encode(c); // Feed the GPS library with the received data

            // Check if the GPS library has detected a valid sentence
            if (gps.location.isValid() || gps.date.isValid() || gps.time.isValid()) {
                hasGPS = true;
                Serial.println("GPS detected and initialized.");
                return;
            }
        }
    }

    Serial.println("GPS not detected. Continuing without GPS.");
}

void initializeMPU() {
      // I2C bus setup
    Wire.begin(RXPin, TXPin); // Set SDA to GPIO 41 and SCL to GPIO 40
    Serial.println("Initializing MPU...");
    // Optional: I2C scan for debugging
    if (mpu.begin()) {
        hasMPU = true;
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
        Serial.println("MPU6050 initialized successfully.");
    } else {
        Serial.println("Failed to initialize MPU6050.");
        hasMPU = false;
    }
}

// --- MPU ---
void readGyroData() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Ensure we are capturing gyro values in degrees per second
    currentDirection = g.gyro.z; // Convert radians to degrees

    // Update accelerometer data
    accelX = a.acceleration.x;
    accelY = a.acceleration.y;
    accelZ = a.acceleration.z;

    // Update gyroscope data
    gyroX = g.gyro.x * 57.2958; // Convert radians to degrees
    gyroY = g.gyro.y * 57.2958;
    gyroZ = g.gyro.z * 57.2958;
}

// --- LED Effects ---
void applyEffectToStrip() {
    if (isMaster) {
        switch (mode) {
            case 0: strip.clear(); break;
            case 1: scrollingRainbow(); break;
            case 2: directionalStrobe(); break;
            case 3: cometTail(); break;
            case 4: twinkleBurst(); break;
            case 5: breathingEffect(); break;
            case 6: gyroRainbowEffect(); break;
        }
        strip.show();
    }
}

void setLEDColorForMode(uint8_t mode) {
    // LED mode logic
    #ifdef PIN_NEOPIXEL
    uint8_t r = 0, g = 0, b = 0;
    switch (mode) {
        case 0: r = 255; g = 0; b = 0; break;
        case 1: r = 0; g = 255; b = 0; break;
        case 2: r = 0; g = 0; b = 255; break;
        case 3: r = 255; g = 255; b = 0; break;
        case 4: r = 0; g = 255; b = 255; break;
        case 5: r = 255; g = 0; b = 255; break;
        case 6: r = 255; g = 255; b = 255; break; // White for Gyro Rainbow Mode
    }
    builtInLED.setPixelColor(0, builtInLED.Color(r, g, b));
    builtInLED.show();
    #endif
}

void scrollingRainbow() {
    uint16_t speedFactor = map(constrain(currentSpeed, 0, 20), 0, 20, 5, 100);
    for (int i = 0; i < NUM_LEDS; i++) {
        uint32_t color = strip.ColorHSV((hue + (i * 65536L / NUM_LEDS)) % 65536, 255, 255);
        strip.setPixelColor(i, color);
    }
    hue += speedFactor;
    if (hue >= 65536) hue -= 65536;
}

void directionalStrobe() {
    uint16_t strobeInterval = map(abs(currentDirection), 0, 180, 100, 10);
    static uint32_t lastUpdate = 0;

    if (millis() - lastUpdate > strobeInterval) {
        for (int i = 0; i < NUM_LEDS; i++) {
            strip.setPixelColor(i, (i % 2 == 0) ? strip.Color(255, 255, 255) : 0);
        }
        lastUpdate = millis();
    } else {
        strip.clear();
    }
}

void cometTail() {
    static int cometPos = 0;
    uint16_t tailLength = map(currentSpeed, 0, 20, 5, 20);

    for (int i = 0; i < NUM_LEDS; i++) {
        if (i == cometPos) {
            strip.setPixelColor(i, strip.Color(0, 255, 255));
        } else if (i < cometPos && cometPos - i <= tailLength) {
            uint8_t brightness = 255 - ((cometPos - i) * (255 / tailLength));
            strip.setPixelColor(i, strip.Color(0, brightness, brightness));
        } else {
            strip.setPixelColor(i, 0);
        }
    }
    cometPos = (cometPos + 1) % NUM_LEDS;
}

void twinkleBurst() {
    uint8_t chanceOfTwinkle = map(abs(currentDirection), 0, 180, 10, 100);
    strip.clear();

    for (int i = 0; i < NUM_LEDS; i++) {
        if (random(0, 255) < chanceOfTwinkle) {
            strip.setPixelColor(i, strip.Color(255, 255, 255));
        }
    }
}

void breathingEffect() {
    static uint16_t brightness = 0;
    static int direction = 1;

    uint16_t breathInterval = map(currentSpeed, 0, 20, 50, 10);
    
    brightness += direction * breathInterval;
    if (brightness >= 255) {
        brightness = 255;
        direction = -1;
    } else if (brightness <= 0) {
        brightness = 0;
        direction = 1;
    }

    for (int i = 0; i < NUM_LEDS; i++) {
        strip.setPixelColor(i, strip.Color(brightness, brightness, brightness));
    }
}

void gyroRainbowEffect() {
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);

    float xRotation = gyro.gyro.x * 57.2958;
    float yRotation = gyro.gyro.y * 57.2958;
    float zRotation = gyro.gyro.z * 57.2958;

    // Cast to int to perform modulo operation
    uint16_t combinedHue = (static_cast<int>(abs(xRotation)) + 
                            static_cast<int>(abs(yRotation)) + 
                            static_cast<int>(abs(zRotation))) * 100 % 65536;

    for (int i = 0; i < NUM_LEDS; i++) {
        uint32_t color = strip.ColorHSV(combinedHue, 255, 255);
        strip.setPixelColor(i, color);
    }
    strip.show();
}

// --- Mode Requirements ---
bool modeRequiresGPS() { return (mode == 1 || mode == 3); }
bool modeRequiresMPU() { return (mode == 2 || mode == 4 || mode == 5 || mode == 6); }