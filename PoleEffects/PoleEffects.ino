#include <Adafruit_NeoPixel.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GPS.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define DATA_PIN 17
#define NUM_LEDS 50
#define BUTTON_PIN 0

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

// MAC addresses of ESP32 devices
uint8_t board1Mac[] = {0xD4, 0xF9, 0x8D, 0x66, 0x12, 0xCA}; // "white" QT Py
uint8_t board2Mac[] = {0xF4, 0x12, 0xFA, 0x59, 0x5B, 0x30}; // "black" QT Py

uint8_t *macAddresses[] = {board1Mac, board2Mac};
const int numBoards = 2;

esp_now_peer_info_t peerInfo;

// Initialize peer communication for all ESP32 devices
void initializePeers() {
  for (int i = 0; i < numBoards; i++) {
    memcpy(peerInfo.peer_addr, macAddresses[i], 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add peer");
    }
  }
}

// Attempt to initialize GPS and MPU sensors
void initializeSensors() {
  if (GPS.begin(0x10)) { // Check if GPS is present
    hasGPS = true;
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    GPS.sendCommand(PGCMD_ANTENNA);
  }
  
  if (mpu.begin()) { // Check if MPU is present
    hasMPU = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  }
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  delay(1000);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  initializePeers();
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  strip.begin();
  strip.setBrightness(50);
  strip.show();

  #ifdef PIN_NEOPIXEL
    pinMode(NEOPIXEL_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_POWER, HIGH);
    builtInLED.begin();
    builtInLED.show();
  #endif

  initializeSensors();
  delay(1000);
}

void loop() {
  if (digitalRead(BUTTON_PIN) == LOW) {
    // Increment mode and send to other boards
    mode = (mode + 1) % 6;
    syncData.mode = mode;
    syncData.hue = hue;

    // Update the LED color for the mode
    setLEDColorForMode(mode);

    // Broadcast the new mode to all peers
    esp_now_send(nullptr, (uint8_t *)&syncData, sizeof(syncData));
    delay(200); // Debounce
  }

  // Set master based on sensor needs and available sensors
  if ((modeRequiresGPS() && hasGPS) || (modeRequiresMPU() && hasMPU)) {
    isMaster = true;
    readSensorData();
    syncData.speed = currentSpeed;
    syncData.direction = currentDirection;
    
    // Periodically send sensor data to other boards
    esp_now_send(nullptr, (uint8_t *)&syncData, sizeof(syncData));
  } else {
    isMaster = false; // Not master if no relevant sensor for the mode
  }

  applyEffectToStrip();
  delay(20);
}

// --- Sensor Data Functions ---
void readSensorData() {
  if (hasGPS) {
    readGPSData();
  }
  if (hasMPU) {
    readGyroData();
  }
}

void readGPSData() {
  while (GPS.available()) {
    GPS.read();
    if (GPS.fix) {
      currentSpeed = GPS.speed * 1.15078; // Convert knots to MPH
      Serial.print("Speed: "); Serial.println(currentSpeed);
    }
  }
}

void readGyroData() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  currentDirection = g.gyro.z * 57.2958; // Convert radians/s to degrees/s
  Serial.print("Direction: "); Serial.println(currentDirection);
}

// --- Mode Requirements ---
bool modeRequiresGPS() { return (mode == 1 || mode == 3); }
bool modeRequiresMPU() { return (mode == 2 || mode == 4); }

// --- LED Color and Effect Functions ---
void setLEDColorForMode(uint8_t mode) {
  switch (mode) {
    case 0: sharedLEDColor(255, 0, 0); break;
    case 1: sharedLEDColor(0, 255, 0); break;
    case 2: sharedLEDColor(0, 0, 255); break;
    case 3: sharedLEDColor(255, 255, 0); break;
    case 4: sharedLEDColor(0, 255, 255); break;
    case 5: sharedLEDColor(255, 0, 255); break;
    default: sharedLEDColor(255, 255, 255); break;
  }
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

void applyEffectToStrip() {
  switch (mode) {
    case 0: strip.clear(); break;
    case 1: scrollingRainbow(); break;
    case 2: directionalStrobe(); break;
    case 3: cometTail(); break;
    case 4: twinkleBurst(); break;
    case 5: breathingEffect(); break;
  }
  strip.show();
}

// --- Effects ---

void scrollingRainbow() {
  uint16_t speedFactor = 10 + (uint16_t)(currentSpeed * 50);
  for (int i = 0; i < NUM_LEDS; i++) {
    uint32_t color = strip.ColorHSV((hue + (i * 65536L / NUM_LEDS)) % 65536, 255, 255);
    strip.setPixelColor(i, color);
  }
  hue += speedFactor;
  if (hue >= 65536) hue -= 65536;
}

void directionalStrobe() {
  uint8_t brightness = (sin(millis() / 200.0) * 127) + 128;
  for (int i = 0; i < NUM_LEDS; i++) {
    int position = (i + (int)(currentDirection / 10)) % NUM_LEDS;
    strip.setPixelColor(position, strip.Color(255, 255, 255, brightness));
  }
}

void cometTail() {
  static int cometPos = 0;
  strip.clear();
  int tailLength = 5;
  for (int i = 0; i < tailLength; i++) {
    int pos = (cometPos - i + NUM_LEDS) % NUM_LEDS;
    uint8_t brightness = 255 - (i * 50);
    strip.setPixelColor(pos, strip.Color(255, 0, 0, brightness));
  }
  cometPos = (cometPos + 1) % NUM_LEDS;
}

void twinkleBurst() {
  strip.clear();
  for (int i = 0; i < NUM_LEDS / 4; i++) {
    int pos = random(NUM_LEDS);
    strip.setPixelColor(pos, strip.Color(random(255), random(255), random(255)));
  }
  delay(50);
}

void breathingEffect() {
  uint8_t brightness = (sin(millis() / 1000.0 * PI) * 127) + 128;
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 255, brightness));
  }
  delay(20);
}

// --- ESP-NOW Callbacks ---

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void onDataRecv(const esp_now_recv_info_t *recvInfo, const uint8_t *incomingData, int len) {
  memcpy(&syncData, incomingData, sizeof(syncData));
  mode = syncData.mode;
  hue = syncData.hue;
  currentSpeed = syncData.speed;
  currentDirection = syncData.direction;

  // Sync LED color based on mode
  sharedLEDColor(syncData.color[0], syncData.color[1], syncData.color[2]);
}