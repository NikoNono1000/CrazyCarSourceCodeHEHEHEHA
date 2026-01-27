#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

#define IR_SENSOR_PIN 34      // Sensor-Ausgang
#define IR_SENSOR_VCC_PIN 32  // Sensor-Versorgung

const char* ssid = "Niko_wifi";         // <-- hier anpassen
const char* password = "N1224M0525R0908!"; // <-- hier anpassen

WebServer server(80);

unsigned long lastSerialPrint = 0;
const unsigned long serialInterval = 200; // ms

void handleRoot() {
  int sensorValue = analogRead(IR_SENSOR_PIN);
  String html = "<html><head><meta http-equiv='refresh' content='0.5'></head><body>";
  html += "<h1>IR Sensor Wert: ";
  html += sensorValue;
  html += "</h1>";
  html += "<p>ESP32 IP: ";
  html += WiFi.localIP().toString();
  html += "</p></body></html>";
  server.send(200, "text/html", html);
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 10; ++i) {
    Serial.println("Serial funktioniert!");
    delay(1000);
  }
  pinMode(IR_SENSOR_PIN, INPUT);
  pinMode(IR_SENSOR_VCC_PIN, OUTPUT);
  digitalWrite(IR_SENSOR_VCC_PIN, HIGH); // 3.3V anlegen (GPIO HIGH)

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  server.on("/", handleRoot);
  server.begin();

  // IP-Adresse mehrfach ausgeben, damit du sie sicher siehst
  for (int i = 0; i < 5; ++i) {
    Serial.print("Verbunden! ESP32 IP: ");
    Serial.println(WiFi.localIP());
    delay(2000);
  }
}

void loop() {
  server.handleClient();

  // Serial-Ausgabe alle 200ms
  if (millis() - lastSerialPrint >= serialInterval) {
    int sensorValue = analogRead(IR_SENSOR_PIN);
    Serial.print("IR Sensor Wert: ");
    Serial.println(sensorValue);
    lastSerialPrint = millis();
  }
}
