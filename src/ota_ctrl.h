#include <ArduinoOTA.h>

// OTA update handler
void setupOTA() {
  // Set hostname for OTA
  String hostname = "esp32-robot-" + String((uint16_t)(ESP.getEfuseMac() >> 32), HEX);
  ArduinoOTA.setHostname(hostname.c_str());
  
  // Set password if desired (optional, remove if not needed)
  // ArduinoOTA.setPassword("admin");
  
  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  ArduinoOTA.onStart([]() {
    String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
    Serial.println("\n[OTA] Start updating " + type);
  });
  
  ArduinoOTA.onEnd([]() {
    Serial.println("[OTA] End");
  });
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("[OTA] Progress: %u%%\r", (progress / (total / 100)));
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("[OTA] Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  
  ArduinoOTA.begin();
  
  if (InfoPrint == 1) {
    Serial.printf("[OTA] Ready - Upload to %s.local\n", hostname.c_str());
  }
}
