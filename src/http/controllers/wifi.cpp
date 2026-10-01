#include "wifi.h"
#include "config/config.h"
#include "http/body_accumulator.h"
#include "http/json_response.h"
#include "../../logger/logger.h"
#include <ArduinoJson.h>
#include <math.h>

void wifiGetController(AsyncWebServerRequest* request) {
  JsonDocument doc;
  doc["wifiTxPower"] = config.wifiTxPower;
  JsonArray arr = doc["wifiList"].to<JsonArray>();
  for (int i = 0; i < config.wifiCount; i++) {
    JsonObject entry = arr.add<JsonObject>();
    entry["ssid"]     = config.wifiList[i].ssid;
    entry["password"] = "";  // 安全起见，不回显密码
  }
  String body;
  serializeJson(doc, body);
  request->send(200, "application/json", body);
}

void wifiPostController(AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
  const char* body = nullptr;
  if (!httpAccumulateBody(request, data, len, index, total, HTTP_JSON_BODY_MAX_BYTES, &body)) return;
  if (body == nullptr) return;

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, body);
  httpReleaseAccumulatedBody(request);
  if (err || !doc["wifiList"].is<JsonArray>()) {
    JsonResp::err(request, 400, "缺少wifiList数组");
    return;
  }

  JsonArray arr = doc["wifiList"].as<JsonArray>();
  if ((int)arr.size() > MAX_WIFI_ENTRIES) {
    JsonResp::err(request, 400, "wifiList超过最大限制（5条）");
    return;
  }

  if (doc["wifiTxPower"].is<float>() || doc["wifiTxPower"].is<int>()) {
    float power = doc["wifiTxPower"].as<float>();
    const float supported[] = {-1.0f, 2.0f, 5.0f, 7.0f, 8.5f, 11.0f,
                               13.0f, 15.0f, 17.0f, 18.5f, 19.0f, 19.5f};
    bool matched = false;
    for (float value : supported) {
      if (fabsf(power - value) < 0.01f) {
        config.wifiTxPower = value;
        matched = true;
        break;
      }
    }
    if (!matched) {
      JsonResp::err(request, 400, "WiFi发射功率不是ESP32支持的档位");
      return;
    }
  }

  int count = 0;
  for (JsonVariant v : arr) {
    if (!v.is<JsonObject>()) continue;
    JsonObject entry = v.as<JsonObject>();
    String ssid = entry["ssid"] | String("");
    if (ssid.length() == 0) continue;
    config.wifiList[count].ssid = ssid;
    String pw = entry["password"] | String("");
    if (pw.length() > 0) config.wifiList[count].password = pw;
    count++;
  }
  config.wifiCount = count;

  ConfigStore::save();
  LOG("HWIFI", "WiFi配置已更新，共 %d 条", config.wifiCount);

  JsonResp::okWithReboot(request, "WiFi配置已保存，设备将在2秒后自动重启");
}
