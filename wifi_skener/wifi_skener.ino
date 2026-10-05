#include "WiFi.h"

void setup() {
  Serial.begin(115200);
  delay(2000);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
}

void loop() {
  Serial.println("\nSkeniram mreze...");
  int n = WiFi.scanNetworks();
  Serial.printf("Nadjeno: %d\n", n);
  for (int i = 0; i < n; i++) {
    Serial.printf("%d: %s (%d dBm) kanal %d %s\n",
      i + 1, WiFi.SSID(i).c_str(), WiFi.RSSI(i), WiFi.channel(i),
      WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "OTVORENA" : "ZASTICENA");
  }
  delay(10000);
}