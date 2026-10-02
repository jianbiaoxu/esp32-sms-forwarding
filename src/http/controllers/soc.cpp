#include "soc.h"
#include <AsyncJson.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <math.h>

void socController(AsyncWebServerRequest* request) {
  AsyncJsonResponse* resp = new AsyncJsonResponse();
  JsonObject root = resp->getRoot();

  root["freeHeap"]     = ESP.getFreeHeap();
  root["minFreeHeap"]  = ESP.getMinFreeHeap();
  root["flashTotal"]   = ESP.getFlashChipSize();
  root["flashUsed"]    = ESP.getSketchSize();
  root["flashFree"]    = ESP.getFlashChipSize() - ESP.getSketchSize();

  if (LittleFS.begin()) {
    root["fsTotal"]    = LittleFS.totalBytes();
    root["fsUsed"]     = LittleFS.usedBytes();
    root["fsFree"]     = LittleFS.totalBytes() - LittleFS.usedBytes();
  }

  root["cpuFreqMHz"]   = ESP.getCpuFreqMHz();

  // ESP32-C3 内置温度传感器读数，仅作为芯片温度参考值。
  float temperatureC = temperatureRead();
  if (!isnan(temperatureC)) {
    root["temperatureC"] = temperatureC;
  }

  // Format chip ID as 12-digit uppercase hex string
  uint64_t mac = ESP.getEfuseMac();
  char chipIdStr[13];
  snprintf(chipIdStr, sizeof(chipIdStr), "%04X%08X",
           (uint32_t)(mac >> 32), (uint32_t)(mac & 0xFFFFFFFF));
  root["chipId"]       = chipIdStr;

  root["uptimeSeconds"] = millis() / 1000;

  resp->setLength();
  request->send(resp);
}
