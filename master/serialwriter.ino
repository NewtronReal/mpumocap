#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>

#define UDP_PORT 4444

const char *ssid = "VDMAEQ720VD";
const char *password = "something";
const char *mdnsName = "vethramdevice";  // This will allow discovery as "esp32.local"

WiFiUDP udp;

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("Configuring access point...");

  // Set up Wi-Fi access point
  if (!WiFi.softAP(ssid, password)) {
    log_e("Soft AP creation failed.");
    while (1);
  }

  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);

  // Start UDP server
  udp.begin(UDP_PORT);
  Serial.print("Listening at: ");
  Serial.println(WiFi.softAPIP());

  // Set up mDNS responder
  if (!MDNS.begin(mdnsName)) {
    Serial.println("Error setting up mDNS responder!");
  } else {
    Serial.print("mDNS responder started. Hostname: ");
    Serial.println(mdnsName);
    Serial.print("You can reach the ESP32 at: ");
    Serial.print(mdnsName);
    Serial.println(".local");
  }
}

void loop() {
  uint8_t buffer[14];
  memset(buffer, 0, 14);
  
  // Check if UDP data is available
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    // Read the incoming UDP data
    udp.read(buffer, 14);
    Serial.write(buffer, 14);  // Output the data for debugging
  }
}
