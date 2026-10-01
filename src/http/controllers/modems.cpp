#include "modems.h"
#include "config/config.h"
#include "http/body_accumulator.h"
#include "http/json_response.h"
#include "../../logger/logger.h"
#include <ArduinoJson.h>

static bool validPin(int pin) {
  return pin >= 0 && pin <= 21;
}

void modemsPostController(AsyncWebServerRequest* request, uint8_t* data,
                          size_t len, size_t index, size_t total) {
  const char* body = nullptr;
  if (!httpAccumulateBody(request, data, len, index, total, HTTP_JSON_BODY_MAX_BYTES, &body)) return;
  if (body == nullptr) return;

  JsonDocument doc;
  bool parseOk = deserializeJson(doc, body) == DeserializationError::Ok
              && doc["modems"].is<JsonArray>();
  httpReleaseAccumulatedBody(request);
  if (!parseOk) {
    JsonResp::err(request, 400, "缺少modems数组或JSON格式错误");
    return;
  }

  JsonArray arr = doc["modems"].as<JsonArray>();
  if ((int)arr.size() > MODEM_COUNT) {
    JsonResp::err(request, 400, "模组数量超过固件支持上限");
    return;
  }

  ModemConfig next[MODEM_COUNT];
  for (ModemId i = 0; i < MODEM_COUNT; i++) next[i] = config.modems[i];

  int position = 0;
  for (JsonVariant value : arr) {
    if (!value.is<JsonObject>() || position >= MODEM_COUNT) continue;
    JsonObject modem = value.as<JsonObject>();
    ModemId id = modem["id"] | position;
    if (!isValidModemId(id)) {
      JsonResp::err(request, 400, "模组编号无效");
      return;
    }
    next[id].enabled = modem["enabled"] | next[id].enabled;
    next[id].name    = modem["name"] | next[id].name;
#ifndef SMS_BOARD_CH343
    next[id].rxPin   = modem["rxPin"] | next[id].rxPin;
    next[id].txPin   = modem["txPin"] | next[id].txPin;
    next[id].enPin   = modem["enPin"] | next[id].enPin;
#endif
    position++;
  }

#ifdef SMS_BOARD_CH343
  // 新板卡只支持固定物理映射，忽略前端或旧客户端提交的 GPIO/EN 字段。
  next[0].rxPin = 20;
  next[0].txPin = 21;
  next[0].enPin = -1;
  next[1].rxPin = 1;
  next[1].txPin = 0;
  next[1].enPin = -1;
#endif

  for (ModemId i = 0; i < MODEM_COUNT; i++) {
#ifndef SMS_BOARD_CH343
    if (!validPin(next[i].rxPin) || !validPin(next[i].txPin) || !validPin(next[i].enPin)) {
      JsonResp::err(request, 400, "GPIO必须在0到21之间");
      return;
    }
    if (next[i].rxPin == next[i].txPin || next[i].rxPin == next[i].enPin || next[i].txPin == next[i].enPin) {
      JsonResp::err(request, 400, "同一路模组的RX、TX、EN不能使用同一个GPIO");
      return;
    }
#endif
    if (next[i].name.length() == 0) next[i].name = "SIM" + String(i + 1);
  }

#ifndef SMS_BOARD_CH343
  for (ModemId i = 0; i < MODEM_COUNT; i++) {
    if (!next[i].enabled) continue;
    for (ModemId j = i + 1; j < MODEM_COUNT; j++) {
      if (!next[j].enabled) continue;
      int pinsI[] = {next[i].rxPin, next[i].txPin, next[i].enPin};
      int pinsJ[] = {next[j].rxPin, next[j].txPin, next[j].enPin};
      for (int a : pinsI) for (int b : pinsJ) {
        if (a == b) {
          JsonResp::err(request, 400, "两路启用模组不能复用同一个GPIO");
          return;
        }
      }
    }
  }
#endif

  for (ModemId i = 0; i < MODEM_COUNT; i++) config.modems[i] = next[i];
  ConfigStore::save();
  LOG("HMODEM", "双模组UART配置已保存，重启后生效");
  JsonResp::okWithReboot(request, "模组UART配置已保存，设备将在2秒后重启");
}
