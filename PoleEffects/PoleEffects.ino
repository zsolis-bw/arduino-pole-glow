#include <Adafruit_NeoPixel.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GPS.h>

#define DATA_PIN 17  // Pin connected to your external LED strip (if available)
#define NUM_LEDS 34  // Adjust if using an external strip
#define BUTTON_PIN 0

// Built-in NeoPixel LED pin for Adafruit QT Py ESP32-S2
#ifdef PIN_NEOPIXEL
  #define BUILTIN_LED_PIN PIN_NEOPIXEL
  Adafruit_NeoPixel builtInLED = Adafruit_NeoPixel(1, BUILTIN_LED_PIN, NEO_GRB + NEO_KHZ800);
#endif

// Set up GPS using I2C (STEMMA QT port)
Adafruit_GPS GPS(&Wire);

// Simulated GPS variables
float simulatedSpeed = 0;  // Simulated speed in MPH (0-10 range with bursts up to 20)
float simulatedDirection = 0;  // Simulated direction (0-360 degrees)
bool useSimulatedGPS = true;  // Toggle between simulated and real GPS data

bool gReverseDirection = false;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, DATA_PIN, NEO_GRB + NEO_KHZ800);

// Variables for effects
uint8_t mode = 0;
uint8_t sharedLEDColor[3] = {0, 0, 0}; // [Red, Green, Blue]
uint16_t hue = 0;  // Global variable for hue (needed for rainbow effect)

// Structure to hold the data for sending/receiving
typedef struct {
  uint8_t mode;
  uint8_t color[3];  // Shared LED color as RGB
  uint16_t hue;      // Hue for rainbow effect
  float speed;       // Speed for effects that depend on it
  float direction;   // Direction for effects that depend on it
} LEDSyncData;

LEDSyncData syncData;

// MAC addresses of each board
uint8_t sparkfunMac[] = {0x34, 0x98, 0x7A, 0x4C, 0xCA, 0x94};
uint8_t qtPyMac[] = {0xD4, 0xF9, 0x8D, 0x66, 0x12, 0xCA};
uint8_t *peerMac;  // Store the peer's MAC dynamically depending on which board is used

esp_now_peer_info_t peerInfo;

// Function to determine if the device is SparkFun ESP32 Thing or QT Py ESP32-S2
void determinePeerMac() {
  uint8_t mac[6];
  WiFi.macAddress(mac);
  if (memcmp(mac, sparkfunMac, 6) == 0) {
    Serial.println("This is the SparkFun ESP32 Thing");
    peerMac = qtPyMac;
  } else if (memcmp(mac, qtPyMac, 6) == 0) {
    Serial.println("This is the Adafruit QT Py ESP32-S2");
    peerMac = sparkfunMac;
  }
}

void setup() {
  // Start serial communication
  Serial.begin(115200);

  // Set up Wi-Fi in station mode for ESP-NOW
  WiFi.mode(WIFI_STA);

  // Wait for Wi-Fi adapter to be ready
  delay(1000);

  // Determine which device we're on
  determinePeerMac();

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register peer
  memcpy(peerInfo.peer_addr, peerMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // Register send and receive callbacks
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  #ifdef PIN_NEOPIXEL
    // Initialize built-in NeoPixel for Adafruit QT Py ESP32-S2
    pinMode(NEOPIXEL_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_POWER, HIGH); // Enable NeoPixel power
    builtInLED.begin();
    builtInLED.show();  // Turn off initially
  #endif

  // Initialize external LED strip (if available)
  strip.begin();
  strip.show();  // Initialize all pixels to 'off'

  // Initialize the GPS (real data can replace simulated when connected)
  GPS.begin(0x10);  // I2C address for PA1010D GPS
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);  // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
}

void loop() {
  // Simulate GPS data or read from GPS module
  if (useSimulatedGPS) {
    simulateGPSData();
  } else {
    readGPSData();
  }

  // Check for button press to change mode
  if (digitalRead(BUTTON_PIN) == LOW) {
    mode = (mode + 1) % 6;  // Cycle through modes
    delay(200);  // Debounce delay

    // Update the sync data and broadcast to the other device
    syncData.mode = mode;
    syncData.hue = hue;       // Sync the hue for effects like rainbow
    syncData.speed = simulatedSpeed;     // Sync the speed
    syncData.direction = simulatedDirection;  // Sync direction
    setLEDColorForMode(mode); // Set the appropriate LED color for the new mode
    memcpy(syncData.color, sharedLEDColor, sizeof(sharedLEDColor));  // Share LED color
    esp_now_send(peerMac, (uint8_t *)&syncData, sizeof(syncData));
  }

  // Display the shared LED color on the built-in LED (if available)
  #ifdef PIN_NEOPIXEL
    builtInLED.setPixelColor(0, builtInLED.Color(sharedLEDColor[0], sharedLEDColor[1], sharedLEDColor[2]));
    builtInLED.show();
  #endif

  // Apply the effect to the LED strip
  applyEffectToStrip();

  delay(20);  // Slight delay to slow down the effect speed
}

// --- LED Color and Effect Functions ---
void setLEDColorForMode(uint8_t mode) {
  switch (mode) {
    case 0:
      sharedLEDColor[0] = 255; sharedLEDColor[1] = 0; sharedLEDColor[2] = 0;  // Red
      break;
    case 1:
      sharedLEDColor[0] = 0; sharedLEDColor[1] = 255; sharedLEDColor[2] = 0;  // Green
      break;
    case 2:
      sharedLEDColor[0] = 0; sharedLEDColor[1] = 0; sharedLEDColor[2] = 255;  // Blue
      break;
    case 3:
      sharedLEDColor[0] = 255; sharedLEDColor[1] = 255; sharedLEDColor[2] = 0;  // Yellow
      break;
    case 4:
      sharedLEDColor[0] = 0; sharedLEDColor[1] = 255; sharedLEDColor[2] = 255;  // Cyan
      break;
    case 5:
      sharedLEDColor[0] = 255; sharedLEDColor[1] = 0; sharedLEDColor[2] = 255;  // Magenta
      break;
    default:
      sharedLEDColor[0] = 255; sharedLEDColor[1] = 255; sharedLEDColor[2] = 255;  // White
      break;
  }
}

void applyEffectToStrip() {
  switch (mode) {
    case 0:
      turnOffLEDs();
      break;
    case 1:
      scrollingRainbow();  // Scrolling rainbow effect based on hue and speed
      break;
    case 2:
      directionalStrobe();  // Pop effect based on sharp direction changes
      break;
    case 3:
      cometTail();  // Comet effect with length adjusting based on speed
      break;
    case 4:
      twinkleBurst();  // Twinkle bursts that increase with speed
      break;
    case 5:
      breathingEffect();  // Breathing light effect based on speed
      break;
  }
  strip.show();
}

void turnOffLEDs() {
  strip.clear();  // Turn off external LED strip
}

void scrollingRainbow() {
  // Example scrolling rainbow effect
  uint16_t speedFactor = 10 + (uint16_t)(syncData.speed * 50);  // Speed scales between slow (~10) to fast (~60)
  for (int i = 0; i < NUM_LEDS; i++) {
    uint32_t color = strip.ColorHSV((syncData.hue + (i * 65536L / NUM_LEDS)) % 65536, 255, 255);
    strip.setPixelColor(i, color);
  }
  syncData.hue += speedFactor;
  if (syncData.hue >= 65536) syncData.hue -= 65536;
}

void directionalStrobe() {
  // Example directional strobe effect (pop effect)
  strip.clear();
  if (abs(simulatedDirection - syncData.direction) > 30) {
    for (int i = 0; i < NUM_LEDS; i++) {
      strip.setPixelColor(i, strip.Color(255, 255, 255));  // White flash
    }
  }
}

void cometTail() {
  // Example comet effect
  strip.clear();
  static uint16_t cometIndex = 0;
  cometIndex = (cometIndex + 1) % NUM_LEDS;
  strip.setPixelColor(cometIndex, strip.Color(255, 255, 255));
}

void twinkleBurst() {
  // Example twinkle effect
  strip.clear();
  for (int i = 0; i < 5; i++) {
    strip.setPixelColor(random(NUM_LEDS), strip.Color(255, 255, 255));
  }
}

void breathingEffect() {
  // Example breathing effect
  static uint8_t brightness = 0;
  brightness = (brightness + 1) % 256;
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, strip.Color(0, 255, 0, brightness));  // Green breathing
  }
}

// --- Simulated and Real GPS Data Functions ---
void simulateGPSData() {
  // Simulate speed and direction changes for testing
  simulatedSpeed = random(0, 11) + (random(0, 5) == 0 ? random(0, 10) : 0);  // Random speed with occasional burst
  simulatedDirection = random(0, 361);  // Random direction between 0 and 360 degrees
}

void readGPSData() {
  // Read actual GPS data from the PA1010D module (if connected)
  GPS.read();
  if (GPS.newNMEAreceived()) {
    GPS.parse(GPS.lastNMEA());
    simulatedSpeed = GPS.speed * 1.15078;  // Convert from knots to MPH
    simulatedDirection = GPS.angle;  // GPS course angle
  }
}

// --- ESP-NOW Callbacks ---
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void onDataRecv(const esp_now_recv_info_t *recvInfo, const uint8_t *incomingData, int len) {
  memcpy(&syncData, incomingData, sizeof(syncData));
  mode = syncData.mode;
  memcpy(sharedLEDColor, syncData.color, sizeof(sharedLEDColor));  // Sync LED color from received data
  Serial.println("Syncing LED strip...");
}
