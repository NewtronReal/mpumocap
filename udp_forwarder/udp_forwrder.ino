#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>

#define WIFIMODE 0
#if WIFIMODE == 0
  const char *ssid = "VDMAEQ720VD";
  const char *password = "something";
#else
  const char *ssid = "Your SSID";
  const char *password = "Your Password";
#endif

const char *mdnsName = "vethramdevice";       // mDNS hostname

#define UDP_RECEIVE_PORT 4444          // Port for receiving UDP packets
#define UDP_FORWARD_PORT 5555          // Port for forwarding UDP packets

WiFiUDP udp;
IPAddress targetIP;                    // Target IP for forwarding (Processing IP)
bool targetIPSet = false;              // Flag to check if target IP is set

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("Setting up Wi-Fi...");

  // Set up Wi-Fi in Access Point mode
  IPAddress myIP;
  #if WIFIMODE == 0
    WiFi.softAP(ssid, password);
    myIP = WiFi.softAPIP();
  #else
    WiFi.begin(ssid,password);
    while(WiFi.status()!=WL_CONNECTED){
      delay(500);
      Serial.print(".");
    }
    myIP = WiFi.localIP();
  #endif
  Serial.println();
  Serial.print("AP IP address: ");
  Serial.println(myIP);

  // Start the UDP service
  udp.begin(UDP_RECEIVE_PORT);
  Serial.println("UDP server started for receiving data");

  // Set up mDNS
  if (!MDNS.begin(mdnsName)) {
    Serial.println("Error setting up mDNS responder!");
  } else {
    Serial.print("mDNS responder started. Hostname: ");
    Serial.print(mdnsName);
    Serial.println(".local");
  }
}

void loop() {
  uint8_t udpBuffer[14];
  int packetSize = udp.parsePacket();

  // Check if we've received a packet
  if (packetSize > 0) {
    // Capture sender's IP and port if not set
    if (!targetIPSet && udp.readString()=="DISCOVER") {
      targetIP = udp.remoteIP();
      targetIPSet = true;
      Serial.print("Discovered target IP: ");
      Serial.println(targetIP);
    }

    // Read the UDP packet into the buffer
    int len = udp.read(udpBuffer, 14);
    if (len > 0) {
      Serial.print("Received UDP packet: ");
      for (int i = 0; i < len; i++) {
        Serial.print(udpBuffer[i], HEX);
        Serial.print(" ");
      }
      Serial.println();

      // Forward the packet to the discovered IP and port if known
      if (targetIPSet) {
        udp.beginPacket(targetIP, UDP_FORWARD_PORT);
        udp.write(udpBuffer, len);
        udp.endPacket();
        Serial.println("Forwarded packet to target IP");
      }
    }
  }
}
