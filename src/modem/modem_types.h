#pragma once

#include <Arduino.h>

// 固件当前支持的 ML307 模组数量。所有按模组隔离的运行态都使用该上限，
// 这样配置、状态接口和业务队列可以共享同一套边界定义。
constexpr uint8_t MODEM_COUNT = 2;
using ModemId = uint8_t;

constexpr ModemId MODEM_PRIMARY = 0;

inline bool isValidModemId(ModemId id) {
  return id < MODEM_COUNT;
}
