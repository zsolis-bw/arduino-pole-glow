#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <Wire.h>
#include <TinyGPSPlus.h> // to-do: move to Adafruit_GPS
// #include <Adafruit_GPS.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "esp_log.h"
#include <unordered_map>
#include <string>
#include <iostream>
#include <vector>
#include <array>

// do you want to synthesize sensors
// #define MOCK_SENSORS true

// Pin and hardware definitions
#define DATA_PIN 17
#define NUM_LEDS 50
#define DEBUG_MODE true

// If we are mocking, we won't rely on hardware
#ifdef MOCK_SENSORS
bool hasGPS = true;
bool hasMPU = true;
#else
bool hasGPS = false;
bool hasMPU = false;
#endif

// List debug functions and their enabled status
std::unordered_map<std::string, bool> debugFunctions = {
    {"GPS_SPEED", true},
    {"GPS_LOC", true},
    {"GPS_ELEV", true},
    {"GPS_SATS", true},
    {"GPS_RAW", false},
    {"GYRO_XYZ", true},
    {"GYRO_ACCEL", false},
    {"MEMORY_SYNC", false}
};

int* buttonPins; // to-do, make this a flexible sized array
int numberOfButtons;

// vars for pin mapping to sensor IO
int RXPin;  // GPS TX -> ESP RX
int TXPin;  // GPS RX -> ESP TX

#define GPSBaud 38400

// GPS and MPU objects
Adafruit_MPU6050 mpu;
TinyGPSPlus gps; // still struggling with the Adafruit lib
HardwareSerial gpsSerial(1); // Use Serial1 for GPS
// Adafruit_GPS GPS(&gpsSerial);

// Other variables
unsigned long lastGPSUpdate = 0;
const unsigned long gpsUpdateInterval = 1000; // Update GPS every 1000 ms
uint8_t mode = 0;
uint16_t hue = 0;
bool isMaster = false;
bool modeSyncInProgress = false;

// NeoPixel and ESP-NOW structures
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, DATA_PIN, NEO_GRB + NEO_KHZ800);

#ifdef PIN_NEOPIXEL
#define BUILTIN_LED_PIN PIN_NEOPIXEL
Adafruit_NeoPixel builtInLED = Adafruit_NeoPixel(1, BUILTIN_LED_PIN, NEO_GRB + NEO_KHZ800);
#endif

typedef struct __attribute__((packed)) {
    float currentDirection;       // Scaled to tenths of degrees
    float currentSpeed;
    float currentLatitude;
    float currentLongitude;
    float currentElevation;
    unsigned int gpsSatellites;
    float gpsHDOP;
} GPSData;

typedef struct __attribute__((packed)) {
    float currentDirection;
    float accelX;          // Scaled to milli-g
    float accelY;          // Scaled to milli-g
    float accelZ;          // Scaled to milli-gxf
    float gyroX;           // Scaled to tenths of degrees/sec
    float gyroY;           // Scaled to tenths of degrees/sec
    float gyroZ;           // Scaled to tenths of degrees/sec
} MPUData;

GPSData gpsData;
MPUData mpuData;

const unsigned long debounceDelay = 50; // 50 ms debounce time
/*
// Array of peer MAC addresses (replace with actual MAC addresses of devices)
uint8_t macAddresses[][6] = {
    //{0xD4, 0xF9, 0x8D, 0x66, 0x12, 0xCA}, // Replace with your first ESP32 MAC
    //{0xF4, 0x12, 0xFA, 0x59, 0x5B, 0x30},  // Replace with your second ESP32 MAC, // Replace with your first ESP32 MAC
    //{0xC8, 0xF0, 0x9E, 0xA8, 0x39, 0xF4}  // dev board
    {0xA0, 0x76, 0x4E, 0x14, 0x6A, 0xF4},
    {0x34, 0x85, 0x18, 0x18, 0x25, 0x74}
};
*/
// C3 macs
// A0:76:4E:14:6A:F4
// 34:85:18:18:25:74

// use a fixed mac address for all devices in the cloud
uint8_t trueMAC[6];
uint8_t newMACAddress[] = {0x6F, 0x1A, 0xA4, 0x07, 0x0D, 0x66};
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

esp_now_peer_info_t peerInfo;

/*
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
*/
// Forward declarations
void debugOutput();
bool isDebugFunctionEnabled(const std::string& functionName);
void setupPins();
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
void sendPairingRequest();
void enterPairingMode(unsigned long duration = 15000); 
void addPeer();
void scanI2CBus();

void setup() {
    Serial.begin(115200);
    delay(2000); // debug needs a sec to show
    // Sensor setup
    setupPins();


    #ifndef MOCK_SENSORS
        initializeGPS();
        if (!hasGPS) { // don't combine MPU and GPS for now
            initializeMPU();
        }
    #else
        Serial.println("Mock mode enabled: Not initializing real sensors.");
        // Just set them as 'detected'
        hasGPS = true;
        hasMPU = true;
    #endif

    WiFi.mode(WIFI_MODE_STA);
    Serial.print("[OLD] ESP32 Board MAC Address:  ");
    Serial.println(WiFi.macAddress());
    esp_wifi_set_mac(WIFI_IF_STA, &newMACAddress[0]);
    Serial.print("[NEW] ESP32 Board MAC Address:  ");
    Serial.println(WiFi.macAddress());


    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // register send callback
    esp_now_register_send_cb(onDataSent);

    // setup peers
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    // Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        Serial.println("Error adding peer");
        return;
    }

    // register recv callback
    esp_now_register_recv_cb(onDataRecv);
}
float randomInRange(float min, float max) {
    return min + (float)rand() / ((float)RAND_MAX / (max - min));
}

void scanI2CBus() {
    Serial.println("Scanning I2C bus...");
    for (byte address = 1; address < 127; ++address) {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0) {
            Serial.print("Found I2C device at address: 0x");
            Serial.println(address, HEX);
        }
    }
}

void loop() {
    // Check all button pins
    for (int i = 0; i < numberOfButtons; i++) {
        if (digitalRead(buttonPins[i]) == LOW) {
            delay(50);
            while (digitalRead(buttonPins[i]) == LOW); // Wait for release
            mode = (mode + 1) % 6; // Change mode
            if (modeRequiresGPS() && hasGPS) {
                isMaster = true;
            } else if (modeRequiresMPU() && hasMPU) {
                isMaster = true;
            } else {
                isMaster = false;
            }
            setLEDColorForMode(mode);
            syncMode();
            // debounce
            delay(200);
        }
    }

        // Generate some pseudo-random or fixed test data
    #ifdef MOCK_SENSORS
        // Generate some pseudo-random or fixed test data
        // gpsData.currentLatitude = 37.7749;     // Example: San Francisco lat
        // gpsData.currentLongitude = -122.4194; // Example: San Francisco lon
        // gpsData.currentSpeed = 10.5;          // Example speed in mph
        // gpsData.currentElevation = 30.0;      // 30 meters elevation
        // gpsData.gpsSatellites = 8;
        // gpsData.gpsHDOP = 1.0;
        //// Randomising the mock data (Saad) 
        gpsData.currentLatitude = randomInRange(-90.0, 90.0);
        gpsData.currentLongitude = randomInRange(-180.0, 180.0);
        gpsData.currentSpeed = randomInRange(0.0, 20.0); // Speed in mph
        gpsData.currentElevation = randomInRange(2500, 4000);    
        gpsData.gpsSatellites = randomInRange(0, 10);
        gpsData.gpsHDOP = 1.0;
        // mpuData.currentDirection = 90.0; // Facing East
        // mpuData.accelX = 0;
        // mpuData.accelY = 0;
        // mpuData.accelZ = 1000;    // 1G upward
        // mpuData.gyroX = 0;
        // mpuData.gyroY = 0;
        // mpuData.gyroZ = 0;
        mpuData.currentDirection = 90.0; // Facing East
        mpuData.accelX = randomInRange(-2000, 2000);      // Acceleration in milli-g
        mpuData.accelY = randomInRange(-2000, 2000);
        mpuData.accelZ = randomInRange(10, 100);    // 1G upward        
        mpuData.gyroX = randomInRange(-150, 250);
        mpuData.gyroY = randomInRange(15, 250);
        mpuData.gyroZ = randomInRange(15, 250);

        // Since sensors are mocked, you can skip calling readGPSData or readGyroData
    #else
        // If not mocking, read the real sensors
        if (hasGPS) {
            readGPSData();
        }
        if (hasMPU) {
            readGyroData();
        }
    #endif

    // Call sendData 
     sendData();

    // Debug output
    if (DEBUG_MODE) {
        debugOutput();
    }
    // Apply LED effects
    applyEffectToStrip();
}

void sendData() {
    // Serial.println("Sending Data");
    static GPSData lastGPSData = {};
    static MPUData lastMPUData = {};
    const unsigned long baseSendInterval = 300; // Base interval in milliseconds
    const unsigned long staggerOffset = isMaster ? 0 : 150; // Master sends first
    static unsigned long lastSendTime = 0;

    if ((millis() - lastSendTime) < (baseSendInterval + staggerOffset)) {
        return; // Skip sending to avoid collision
    }
    lastSendTime = millis();

    // Send GPS Data
    if (hasGPS) {
        GPSData currentGPSData = gpsData;

        // Check if the current GPS data differs from the last GPS data sent
        if (memcmp(&currentGPSData, &lastGPSData, sizeof(GPSData)) != 0) {
            esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&currentGPSData, sizeof(currentGPSData));
if (result == ESP_OK) {
                memcpy(&lastGPSData, &currentGPSData, sizeof(GPSData));
                if(isDebugFunctionEnabled("MEMORY_SYNC")){
                    Serial.println("GPS data sent successfully.");
                }
            } else {
                Serial.println("Error sending GPS data.");
            }
        }
        // If data hasn't changed, don't send.
    }

    // symc MPU data if necessary
    if (hasMPU) {
        MPUData currentMPUData = mpuData;

        // Check if the current MPU data is different from the last sent data
        if (memcmp(&currentMPUData, &lastMPUData, sizeof(MPUData)) != 0) {
            esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&currentMPUData, sizeof(currentMPUData));
            if (result == ESP_OK) {
                memcpy(&lastMPUData, &currentMPUData, sizeof(MPUData));
                if(isDebugFunctionEnabled("MEMORY_SYNC")){
                    Serial.println("MPU data sent successfully.");
                }
            } else {
                Serial.println("Error sending MPU data.");
            }
        }
        // If the data hasn't changed, do nothing
    }
}

void setupPins() {
    #ifdef ARDUINO_ESP32_DEV
        buttonPins =  new int[2] {0, 18}; 
        numberOfButtons = sizeof(buttonPins) / sizeof(buttonPins[0]);
        RXPin = 14;
        TXPin = 12;
        Serial.println("Board: ESP32 Dev Module");
    #elif defined(ARDUINO_ADAFRUIT_QTPY_ESP32S2)
        buttonPins =  new int[2] {0, 18}; 
        numberOfButtons = sizeof(buttonPins) / sizeof(buttonPins[0]);
        RXPin = 41;
        TXPin = 40;
        Serial.println("Board: ESP32-S2");
    #elif defined(ARDUINO_ADAFRUIT_QTPY_ESP32C3)
        buttonPins =  new int[2] {9, 18}; 
        numberOfButtons = sizeof(buttonPins) / sizeof(buttonPins[0]);
        RXPin = 5;
        TXPin = 6;
        Serial.println("Board: ESP32-C3");
    #elif defined(ARDUINO_ESP32_THING)
        buttonPins =  new int[2] {0, 18}; 
        numberOfButtons = sizeof(buttonPins) / sizeof(buttonPins[0]);
        RXPin = 15;
        TXPin = 2;
        Serial.println("Board: SparkFun ESP32 Thing");
    #else
        buttonPins =  new int[2] {0, 18}; 
        numberOfButtons = sizeof(buttonPins) / sizeof(buttonPins[0]);
        RXPin = 41; // Default fallback
        TXPin = 40;
        Serial.println("Board: Unknown, using default pins");
    #endif

    // Configure button pins as inputs
    for (int i = 0; i < numberOfButtons; i++) {
        Serial.printf("Enabling pin %d (GPIO %d)\n", i, buttonPins[i]);
        pinMode(buttonPins[i], INPUT_PULLUP);
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
// Arduiono IDE compatible version
// void onDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
        //const uint8_t *mac_addr = info->src_addr; 

            // Check if the incoming data size matches the size of 'mode'
        if (len == sizeof(mode)) {
            uint8_t receivedMode;
            memcpy(&receivedMode, incomingData, sizeof(receivedMode));

            // If the received mode is different from the current mode, update it
            if (receivedMode != mode) {
                mode = receivedMode;
                Serial.printf("Mode synchronized to: %d\n", mode);
                setLEDColorForMode(mode);
            }
            return; // We've handled the mode update, so just return
        }

        if (!hasGPS && len == sizeof(GPSData)) {
        //if (len == sizeof(GPSData)) {
            GPSData *receivedGPS = (GPSData *)incomingData;
            gpsData = *receivedGPS;
            if ( isDebugFunctionEnabled("MEMORY_COPY") ) {
            Serial.printf("Received GPS Data: Lat: %f, Lon: %f, Speed: %f\n",
                      gpsData.currentLatitude, gpsData.currentLongitude, gpsData.currentSpeed);
            }
        }
        if (!hasMPU && len == sizeof(MPUData)) {
        //if (len == sizeof(MPUData)) {
            MPUData *receivedMPU = (MPUData *)incomingData;
            mpuData = *receivedMPU;
            if ( isDebugFunctionEnabled("MEMORY_COPY") ) {
              Serial.printf("Received MPU Data: GyroX: %f, GyroY: %f, GyroZ: %f\n", 
                              mpuData.gyroX, mpuData.gyroY, mpuData.gyroZ);
            }
        }
    
}

void syncMode() {
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&mode, sizeof(mode));
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
        
        if (memcmp(mac_addr, broadcastAddress, 6) == 0) {
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

    if((modeRequiresGPS() || mode == 0)){
        if (isDebugFunctionEnabled("GPS_SPEED")) {
            Serial.print("Speed (mph): ");
            Serial.println(gpsData.currentSpeed);
        } 
        if (isDebugFunctionEnabled("GPS_LOC")) {
            Serial.print("Latitude: ");
            Serial.println(gpsData.currentLatitude, 6);
            Serial.print("Longitude: ");
            Serial.println(gpsData.currentLongitude, 6);
        } 
        if (isDebugFunctionEnabled("GPS_ELEV")) {
            Serial.print("Elevation (meters): ");
            Serial.println(gpsData.currentElevation);
        } 
        if (isDebugFunctionEnabled("GPS_SATS")) {
            Serial.print("Satellites: ");
            Serial.println(gpsData.gpsSatellites);
        }
    }
    // gyro functions
    if((modeRequiresMPU() || mode == 0)){
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
    while (gpsSerial.available() > 0) {
        char c = gpsSerial.read();
        gps.encode(c); // Feed TinyGPS++ library

        if (gps.location.isUpdated()) {
            gpsData.currentLatitude = gps.location.lat();
            gpsData.currentLongitude = gps.location.lng();
        }
        if (gps.altitude.isUpdated()) {
            gpsData.currentElevation = gps.altitude.meters();
        }
        if (gps.speed.isUpdated()) {
            gpsData.currentSpeed = gps.speed.mph() > 0.5 ? gps.speed.mph() : 0.0;
        }
        if (gps.satellites.isUpdated()) {
            gpsData.gpsSatellites = gps.satellites.value();
        }
        if (gps.hdop.isUpdated()) {
            gpsData.gpsHDOP = gps.hdop.value();
        }
    }
}

void initializeGPS() {
    Serial.println("Initializing GPS...");
    hasGPS = false; // Assume GPS is not present initially
    gpsSerial.setRxBufferSize(1024);
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
    Serial.println("Initializing MPU...");
    Wire.begin(RXPin, TXPin); // Ensure correct SDA and SCL pins
    if (mpu.begin(0x68)) { // Use 0x68 or 0x69 based on your I2C scan
        hasMPU = true;
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
        Serial.println("MPU6050 initialized successfully.");
    } else {
        Serial.println("Failed to initialize MPU6050. Check connections and address.");
        hasMPU = false;
    }
}

// --- MPU ---
void readGyroData() {
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);

    mpuData.currentDirection = gyro.gyro.z * 57.2958; // Convert radians to degrees
    mpuData.accelX = accel.acceleration.x * 1000; // Scale to milli-g
    mpuData.accelY = accel.acceleration.y * 1000;
    mpuData.accelZ = accel.acceleration.z * 1000;
    mpuData.gyroX = gyro.gyro.x * 57.2958; // Convert radians to degrees/sec
    mpuData.gyroY = gyro.gyro.y * 57.2958;
    mpuData.gyroZ = gyro.gyro.z * 57.2958;
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
    uint16_t speedFactor = map(constrain(gpsData.currentSpeed, 0, 20), 0, 20, 5, 100);
    for (int i = 0; i < NUM_LEDS; i++) {
        uint32_t color = strip.ColorHSV((hue + (i * 65536L / NUM_LEDS)) % 65536, 255, 255);
        strip.setPixelColor(i, color);
    }
    hue += speedFactor;
    if (hue >= 65536) hue -= 65536;
}

void directionalStrobe() {
    uint16_t strobeInterval = map(abs(mpuData.currentDirection), 0, 180, 100, 10);
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
    uint16_t tailLength = map(gpsData.currentSpeed, 0, 20, 5, 20);

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
    uint8_t chanceOfTwinkle = map(abs(mpuData.currentDirection), 0, 180, 10, 100);
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

    uint16_t breathInterval = map(gpsData.currentSpeed, 0, 20, 50, 10);
    
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