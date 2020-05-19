#include <WiFi.h>

const char* ssid = "NETGEAR70";
const char* password =  "aquatickayak304";

const uint16_t port = 8091;
const char *host = "192.168.1.2";

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  Serial.println(WiFi.localIP());
}

void loop() {

  WiFiClient client;
  if(!client.connect(host,port)){
      Serial.println("Connection to host failed");
      delay(1000);
      return;
      }
      float a = 108.25;
      client.flush();
      client.print(a);
      delay(100);
}

    //client.stop();
    //Serial.println("Client disconnected");

  
