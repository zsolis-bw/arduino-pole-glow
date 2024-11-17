#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GPS.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define DATA_PIN 17
#define NUM_LEDS 50
#define BUTTON_PINS {0, 18} // Array of GPIO pins to use for buttons
#define DEBUG_MODE true // Toggle to enable or disable debug mode
#define DEBUG_FUNCTIONS {"GPS_SPEED", "GPS_LOC", "GYRO_XYZ"} // List of functions to monitor TO-DO: Implement

const int buttonPins[] = BUTTON_PINS;
const int numberOfButtons = sizeof(buttonPins) / sizeof(buttonPins[0]);

// Define GPS and MPU-6050 objects
Adafruit_GPS GPS(&Wire);
Adafruit_MPU6050 mpu;

float currentSpeed = 0;
float currentDirection = 0;
uint8_t mode = 0;
uint16_t hue = 0;
bool isMaster = false;
bool hasGPS = false;
bool hasMPU = false;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, DATA_PIN, NEO_GRB + NEO_KHZ800);

#ifdef PIN_NEOPIXEL
  #define BUILTIN_LED_PIN PIN_NEOPIXEL
  Adafruit_NeoPixel builtInLED = Adafruit_NeoPixel(1, BUILTIN_LED_PIN, NEO_GRB + NEO_KHZ800);
#endif

typedef struct {
  uint8_t mode;
  uint8_t color[3];
  uint16_t hue;
  float speed;
  float direction;
} LEDSyncData;

LEDSyncData syncData;

// MAC addresses of all ESP32 devices
uint8_t macAddresses[][6] = {
  {0xD4, 0xF9, 0x8D, 0x66, 0x12, 0xCA},
  {0xF4, 0x12, 0xFA, 0x59, 0x5B, 0x30}
};

esp_now_peer_info_t peerInfo;

// Debug functions array
const char* debugFunctions[] = DEBUG_FUNCTIONS;

// Forward declarations for functions
void initializePeers();
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len);
void setLEDColorForMode(uint8_t mode);
void applyEffectToStrip();
void readSensorData();
void readGPSData();
void readGyroData();
bool modeRequiresGPS();
bool modeRequiresMPU();
void scrollingRainbow();
void directionalStrobe();
void cometTail();
void twinkleBurst();
void breathingEffect();
void gyroRainbowEffect();
void sharedLEDColor(uint8_t r, uint8_t g, uint8_t b);
void debugOutput(const char* function);

// Initialize peer communication for all ESP32 devices
void initializePeers() {
  for (int i = 0; i < sizeof(macAddresses) / sizeof(macAddresses[0]); i++) {
    memcpy(peerInfo.peer_addr, macAddresses[i], 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add peer");
    }
  }
}

void setup() {
    Serial.begin(115200);
    delay(1000); // Initial delay to allow the serial monitor to initialize

    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    initializePeers();
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);

    // Initialize button pins
    for (int i = 0; i < numberOfButtons; i++) {
        pinMode(buttonPins[i], INPUT_PULLUP);
    }

    strip.begin();
    strip.setBrightness(50);
    strip.show();

    #ifdef PIN_NEOPIXEL
        pinMode(NEOPIXEL_POWER, OUTPUT);
        digitalWrite(NEOPIXEL_POWER, HIGH);
        builtInLED.begin();
        builtInLED.show();
    #endif

    // Custom I2C pin configuration
    Wire.begin(41, 40); // Set SDA to GPIO 41 and SCL to GPIO 40

    // Only initialize the GPS if this device is supposed to have one
    if (hasGPS) {
        if (GPS.begin(0x10)) {
            GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
            GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
            GPS.sendCommand(PGCMD_ANTENNA);
            Serial.println("GPS initialized successfully.");
        } else {
            Serial.println("Failed to initialize GPS.");
            hasGPS = false;
        }
    }

    // Only initialize the MPU-6050 if this device is supposed to have one
    if (hasMPU) {
        if (mpu.begin()) {
            mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
            mpu.setGyroRange(MPU6050_RANGE_500_DEG);
            mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
            Serial.println("MPU6050 initialized successfully.");
        } else {
            Serial.println("Failed to initialize MPU6050.");
            hasMPU = false;
        }
    }

    delay(1000); // Delay to ensure all components are properly initialized
}

void loop() {
    // Button press logic remains the same
    for (int i = 0; i < numberOfButtons; i++) {
        if (digitalRead(buttonPins[i]) == LOW) {
            mode = (mode + 1) % 7;
            syncData.mode = mode;
            esp_now_send(nullptr, (uint8_t *)&syncData, sizeof(syncData));
            setLEDColorForMode(mode);
            delay(200); // Debounce delay
        }
    }

    // Only handle GPS if this device has it
    if (hasGPS && modeRequiresGPS()) {
        readGPSData();
    }

    // Only handle MPU-6050 if this device has it
    if (hasMPU && modeRequiresMPU()) {
        readGyroData();
    }

    applyEffectToStrip();
    delay(20);
}

// Debug function to output sensor information selectively
void debugOutput(const char* function) {
  if (strcmp(function, "GPS_SPEED") == 0 && hasGPS) {
    Serial.print("Current Speed: ");
    Serial.println(currentSpeed);
  }
  if (strcmp(function, "GPS_LOC") == 0 && hasGPS) {
    // Add more GPS-specific debug output if needed
    Serial.println("Debugging GPS Location...");
  }
  if (strcmp(function, "GYRO_XYZ") == 0 && hasMPU) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    Serial.print("Gyro X: ");
    Serial.print(g.gyro.x);
    Serial.print(" rad/s, Y: ");
    Serial.print(g.gyro.y);
    Serial.print(" rad/s, Z: ");
    Serial.println(g.gyro.z);
  }
}

// --- Sensor Data Functions ---
void readSensorData() {
  if (hasGPS) {
    readGPSData();
    delay(10); // Small delay to prevent I2C bus collisions
  }
  if (hasMPU) {
    readGyroData();
    delay(10); // Small delay to prevent I2C bus collisions
  }
}

void readGPSData() {
    while (GPS.available()) {
        GPS.read();
        if (GPS.fix) {
            currentSpeed = GPS.speed * 1.15078;
        }
    }
}

void readGyroData() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Ensure we are capturing gyro values in degrees per second
    currentDirection = g.gyro.z * 57.2958; // Convert radians to degrees

    if (DEBUG_MODE) {
        // Print gyro data for debugging purposes
        Serial.print("Gyro X: ");
        Serial.print(g.gyro.x * 57.2958);
        Serial.print(" deg/s, Y: ");
        Serial.print(g.gyro.y * 57.2958);
        Serial.print(" deg/s, Z: ");
        Serial.println(g.gyro.z * 57.2958);
    }
}

// --- Mode Requirements ---
bool modeRequiresGPS() { return (mode == 1 || mode == 3); }
bool modeRequiresMPU() { return (mode == 2 || mode == 4); }

// --- LED Color and Effect Functions ---
void setLEDColorForMode(uint8_t mode) {
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

void applyEffectToStrip() {
  switch (mode) {
    case 0: strip.clear(); break;
    case 1: scrollingRainbow(); break;
    case 2: directionalStrobe(); break;
    case 3: cometTail(); break;
    case 4: twinkleBurst(); break;
    case 5: breathingEffect(); break;
    case 6: gyroRainbowEffect(); break; // New gyroscope-based rainbow effect
  }
  strip.show();
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
      if (i % 2 == 0) {
        strip.setPixelColor(i, strip.Color(255, 255, 255));
      } else {
        strip.setPixelColor(i, 0);
      }
    }
    lastUpdate = millis();
  } else {
    strip.clear();
  }
}

void cometTail() {
  uint16_t tailLength = map(currentSpeed, 0, 20, 5, 20);
  static int cometPos = 0;

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
  // Check if the MPU6050 is available
  if (!mpu.begin()) {
    Serial.println("Reinitializing MPU6050...");
    if (!mpu.begin()) {
      Serial.println("Failed to reinitialize MPU6050.");
      return;
    }
    Serial.println("MPU6050 reinitialized successfully.");
  }

  // Get sensor events
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // Convert gyro values from radians to degrees
  float xRotation = gyro.gyro.x * 57.2958;
  float yRotation = gyro.gyro.y * 57.2958;
  float zRotation = gyro.gyro.z * 57.2958;

  // Use acceleration values directly
  float xAccel = accel.acceleration.x;
  float yAccel = accel.acceleration.y;
  float zAccel = accel.acceleration.z;

  // Debug output for sensor readings
  if (DEBUG_MODE) {
    Serial.print("Gyro X: ");
    Serial.print(xRotation);
    Serial.print(", Y: ");
    Serial.print(yRotation);
    Serial.print(", Z: ");
    Serial.println(zRotation);

    Serial.print("Accel X: ");
    Serial.print(xAccel);
    Serial.print(", Y: ");
    Serial.print(yAccel);
    Serial.print(", Z: ");
    Serial.println(zAccel);
  }

  // Normalize the values to a 0-65535 range for hue
  uint16_t hueX = (uint16_t)(abs(xRotation) * 100) % 65536;
  uint16_t hueY = (uint16_t)(abs(yRotation) * 100) % 65536;
  uint16_t hueZ = (uint16_t)(abs(zRotation) * 100) % 65536;

  uint16_t accelHueX = (uint16_t)(abs(xAccel) * 5000) % 65536;
  uint16_t accelHueY = (uint16_t)(abs(yAccel) * 5000) % 65536;
  uint16_t accelHueZ = (uint16_t)(abs(zAccel) * 5000) % 65536;

  // Combine all hues
  uint16_t combinedHue = (hueX + hueY + hueZ + accelHueX + accelHueY + accelHueZ) / 6;

  // Debug combined hue
  if (DEBUG_MODE) {
    Serial.print("Combined Hue: ");
    Serial.println(combinedHue);
  }

  // Set the color across the LED strip using the combined hue
  for (int i = 0; i < NUM_LEDS; i++) {
    uint32_t color = strip.ColorHSV(combinedHue, 255, 255); // Full saturation and brightness
    strip.setPixelColor(i, color);
  }
  strip.show();
}

// --- ESP-NOW Callbacks ---
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  memcpy(&syncData, incomingData, sizeof(syncData));
  mode = syncData.mode;
  hue = syncData.hue;
  currentSpeed = syncData.speed;
  currentDirection = syncData.direction;
  setLEDColorForMode(mode);
  sharedLEDColor(syncData.color[0], syncData.color[1], syncData.color[2]);
}

void sharedLEDColor(uint8_t r, uint8_t g, uint8_t b) {
  syncData.color[0] = r;
  syncData.color[1] = g;
  syncData.color[2] = b;
  #ifdef PIN_NEOPIXEL
    builtInLED.setPixelColor(0, builtInLED.Color(r, g, b));
    builtInLED.show();
  #endif
}