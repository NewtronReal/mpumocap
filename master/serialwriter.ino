#include <WiFi.h>
#include <NetworkClient.h>
#include <WiFiAP.h>
#include <WiFiUdp.h>

#define UDP_PORT 4444

const char *ssid = "VDMAEQ720VD";
const char *password = "something";

NetworkServer server(80);

WiFiUDP udp;

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("Configuring access point...");

  if (!WiFi.softAP(ssid, password)) {
    log_e("Soft AP creation failed.");
    while (1);
  }
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  server.begin();
  Serial.println("Server started");
  udp.begin(UDP_PORT);
  Serial.print("Listenint at:");
  Serial.println(WiFi.softAPIP());
}

void loop() {
  uint8_t buffer[14];
  memset(buffer,0,14);
  udp.parsePacket();
  if(udp.read(buffer,14)>0){
    if(char(buffer[0]=='r')){

    }
    Serial.write(buffer,14);
  }
}
