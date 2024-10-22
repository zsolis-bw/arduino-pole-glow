#include "WiFi.h"

void setup() {
  // Start the serial communication
  Serial.begin(115200);

  // Wait for 5 seconds to ensure everything is initialized
  delay(5000);

  // Initialize WiFi (even if you're not connecting, this enables access to MAC)
  WiFi.mode(WIFI_STA); // Set ESP32 to station mode

  // Wait for Wi-Fi adapter to be ready
  delay(1000);

  // Print the MAC address to the Serial Monitor
  Serial.print("ESP32 MAC Address: ");
  Serial.println(WiFi.macAddress());
}

void loop() {
  // Do nothing in the loop
}
